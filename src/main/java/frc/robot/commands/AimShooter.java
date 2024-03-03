// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterConstants;
import frc.robot.Utility;
import frc.robot.subsystems.ShooterTilt;
import java.lang.Math;

public class AimShooter extends Command {
  private final ShooterTilt m_shooterTilt;

  private boolean detected = false;

  /** Creates a new AimShooter. */
  public AimShooter(ShooterTilt shooterTilt) {
    m_shooterTilt = shooterTilt;
    addRequirements(m_shooterTilt);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterTilt.check();

    if (!detected) {
      // Test for april tag in view and correct speaker ID
      if (!Utility.aprilTagInView()) { return; }
      if (!Utility.testShooterID())  { return; }
  
      detected = true;
      double angleGoal = calcShooterAngle();
      m_shooterTilt.setGoal(angleGoal); // Calculate and aim shooter
    }
  }

  private double calcShooterAngle() {
    Pose3d pos = Utility.aprilTagPos();

    double y = -pos.getY();
    double z = pos.getZ();

    // Gets the robot's shooter tilt angle needed to face the tag directly
    double angleOffsetFromTag = ShooterConstants.limelightAngle - Math.atan(y / z);
    // Angle above the line from the limelight to the AprilTag
    double angleAboveTag = Math.PI - angleOffsetFromTag;

    // Pythagorean theorem to find depth to AprilTag from the camera
    double depthToTag = Math.sqrt(Math.pow(y, 2) + Math.pow(z, 2));

    // USING the above values to calculate the angle offset from the AprilTag
    // to the speaker, using law of cosines and then law of sines:

    // Find the length from limelight to the speaker based on the above and the law of cosines
    double lengthToSpeaker = Math.sqrt(
      (Math.pow(depthToTag, 2) + Math.pow(ShooterConstants.distTagToSpeaker, 2)) -
      (2 * depthToTag * ShooterConstants.distTagToSpeaker * Math.cos(angleAboveTag))
    );

    // Using law of sines, find the angle offset needed to aim the arm from the tag to the speaker instead
    double angleOffset = Math.asin(
      (Math.sin(angleAboveTag) / lengthToSpeaker) * ShooterConstants.distTagToSpeaker
    );

    // Angle up from horizontal to face the speaker directly
    double directAngle = (Math.PI / 2) - (angleOffsetFromTag - angleOffset);

    // Find the horizontal distance from the robot to the wall based on this angle and the lengthToSpeaker
    double distHorizontal = lengthToSpeaker * Math.cos(directAngle);
    double distVertical = lengthToSpeaker * Math.sin(directAngle);

    System.out.println("Horizontal distance: ");
    System.out.println(distHorizontal);
    System.out.println("Vertical distance: ");
    System.out.println(distHorizontal);

    System.out.println("Original angle: ");
    System.out.println(((Math.PI / 2) - directAngle) * (180 / Math.PI));

    // Use this equation (https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y))
    // to calculate the angle needed
    double velSquared = Math.pow(ShooterConstants.initialVel, 2);
    double underRadical = (
      Math.pow(velSquared, 2) - 
      ShooterConstants.gravity *
      (ShooterConstants.gravity * Math.pow(distHorizontal, 2) + 2 * distVertical * velSquared)
    );

    if (underRadical < 0) {
      System.out.println("SPEAKER OUT OF RANGE. Update the initial velocity or move closer!");
      return directAngle;
    }

    double angle = Math.atan(
      (velSquared - Math.sqrt(underRadical)) /
      (ShooterConstants.gravity * distHorizontal)
    );

    System.out.println("New angle: ");
    System.out.println(90 - (angle * (180 / Math.PI)));

    SmartDashboard.putNumber("Suggested Arm Angle", angle);
    return angle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    detected = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stops when the shooter tilted to the correct angle
    // after the AprilTag was detected
    return detected && m_shooterTilt.atGoal();
    // return false;
  }
}

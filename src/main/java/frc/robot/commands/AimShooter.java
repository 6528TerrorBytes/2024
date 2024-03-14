// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utility;
import frc.robot.subsystems.ShooterTilt;
import java.lang.Math;

public class AimShooter extends Command {
  private final ShooterTilt m_shooterTilt;

  private boolean detected = false;
  
  private boolean m_endAtGoal = false;

  /** Creates a new AimShooter. */
  public AimShooter(ShooterTilt shooterTilt, boolean endAtGoal) {
    m_shooterTilt = shooterTilt;
    m_endAtGoal = endAtGoal;
    addRequirements(m_shooterTilt);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterTilt.check();
    m_shooterTilt.testSwitches();

    // Test for april tag in view
    if (!Utility.aprilTagInView()) { return; }

    detected = true;
    double angleGoal = calcShooterAngle();
    if (angleGoal <= 0 || angleGoal >= 70) {
      System.out.println("PAST 70");
      System.out.println(angleGoal);
      
      Pose3d pos = Utility.aprilTagPos();
      System.out.println(-pos.getY());
      System.out.println(pos.getZ());

      angleGoal = 45;
    }
    m_shooterTilt.setGoal(angleGoal); // Calculate and aim shooter
  }

  private double calcShooterAngle() {
    // All this math calculated here:`  
    // https://www.desmos.com/calculator/svw50rmopy

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

    // Calculates the angle, finds the error when accounting for the shooter length,
    // and then recalculates accounting for the error
    double error = 0;
    double angle = 0;
    for (int i = 0; i < 4; i++) {
      angle = angleToPoint(distHorizontal, distVertical - error);

      if (angle < 0) { // Speaker not in range
        detected = false;
        return ShooterConstants.angleAtVertical; // Put arm up
      }

      error += findNoteHeight(angle, distHorizontal) - distVertical;
    }

    angle = 90 - (angle * (180 / Math.PI));
    angle -= 90 - ShooterConstants.encoderAngleToHorizontal;

    SmartDashboard.putNumber("Suggested Arm Angle", angle);
    return angle;
  }

  // Returns the angle to point a shooter, where x and y are in meters
  // Uses particle motion to calculate the angle, accounting for gravity
  private double angleToPoint(double x, double y) {
    // Use this equation (https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y))
    // to calculate the angle needed
    double velSquared = Math.pow(ShooterConstants.initialVel, 2);
    double underRadical = (
      Math.pow(velSquared, 2) - 
      ShooterConstants.gravity *
      (ShooterConstants.gravity * Math.pow(x, 2) + 2 * y * velSquared)
    );

    if (underRadical < 0) {
      System.out.println("SPEAKER OUT OF RANGE. Update the initial velocity or move closer!");
      return -1;
    }

    return Math.atan(
      (velSquared - Math.sqrt(underRadical)) /
      (ShooterConstants.gravity * x)
    );
  }

  // Finds the height of the note at an x distance in meters
  // when the shooter is pointed at the given angle
  private double findNoteHeight(double angle, double x) {
    return ( // See line 41 of https://www.desmos.com/calculator/svw50rmopy
      (x - (ShooterConstants.shooterLength * Math.cos(angle))) * Math.tan(angle) -
      ((ShooterConstants.gravity * Math.pow((x - (ShooterConstants.shooterLength * Math.cos(angle))), 2)) /
       (2 * Math.pow(ShooterConstants.initialVel, 2) * Math.pow(Math.cos(angle), 2)))
      + ShooterConstants.shooterLength * Math.sin(angle) 
    );
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
    // return detected && m_shooterTilt.atGoal();
    return m_endAtGoal & detected & m_shooterTilt.atGoal();
  }
}

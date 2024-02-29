// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.ShooterTilt;
import java.lang.Math;

public class AimShooter extends Command {
  private final ShooterTilt m_shooterTilt;

  private boolean atHorizontal = false;
  private boolean detected = false;

  /** Creates a new AimShooter. */
  public AimShooter(ShooterTilt shooterTilt) {
    m_shooterTilt = shooterTilt;
    addRequirements(m_shooterTilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterTilt.setGoal(Constants.ShooterConstants.limelightDetectAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atHorizontal = m_shooterTilt.atGoal();
    SmartDashboard.putBoolean("athorizontal", atHorizontal);
    if (!atHorizontal) { return; }

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

    // Since the limelight is horizontal when the shooter is 88 degrees, this is the small offset needed
    double angleFromHorizontal = 90 - Constants.ShooterCommand.limelightHorizontal;
    // The angle of the shooter adjusted for the angle from horizontal in radians
    double currentAngle = (m_shooterTilt.getAngle() + angleFromHorizontal) * (Math.PI / 180);

    // Gets the robot's shooter tilt angle needed to face the tag directly
    double angleOffsetFromTag = currentAngle - Math.atan(y / z);
    // Angle above the line from the limelight to the AprilTag
    double angleAboveTag = Math.PI - angleOffsetFromTag;

    // Pythagorean theorem to find depth to AprilTag from the camera
    double depthToTag = Math.sqrt(Math.pow(y, 2) + Math.pow(z, 2));

    // USING the above values to calculate the angle offset from the AprilTag
    // to the speaker, using law of cosines and then law of sines:

    // Find the length from limelight to the speaker based on the above and the law of cosines
    double lengthToSpeaker = Math.sqrt(
      (Math.pow(depthToTag, 2) + Math.pow(Constants.ShooterConstants.distTagToSpeaker, 2)) -
      (2 * depthToTag * Constants.ShooterConstants.distTagToSpeaker * Math.cos(angleAboveTag))
    );

    // Using law of sines, find the angle offset needed to aim the arm from the tag to the speaker instead
    double angleOffset = Math.asin(
      (Math.sin(angleAboveTag) / lengthToSpeaker) * Constants.ShooterConstants.distTagToSpeaker
    ) * (180 / Math.PI);

    // Angle of the arm to face the speaker!
    double angle = angleOffsetFromTag * (180 / Math.PI) - angleOffset;

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

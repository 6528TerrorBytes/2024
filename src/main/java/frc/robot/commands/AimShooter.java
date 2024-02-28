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

    double y = -pos.getY(); // Constants.ShooterConstants.distTagToSpeaker
    double z = pos.getZ();

    // For convenience, the angle that the shooter is at to detect the tag in radians:
    double angleSet = Constants.ShooterConstants.limelightDetectAngle * (Math.PI / 180);
    double wallAngle = Math.PI - angleSet;

    // Vertical distance from the center of the speaker the limelight faces up to the AprilTag
    double heightToTag = y / Math.sin(angleSet);
    // Added distance from the AprilTag to the speaker center, where the arm should aim
    double heightToSpeaker = heightToTag + Constants.ShooterConstants.distTagToSpeaker;

    // Distance from the limelight to the wall
    double depthToWall = z - (heightToTag * Math.cos(angleSet));

    // Length from the limelight to the speaker center, using law of cosines
    double lengthToSpeaker = Math.sqrt(
        (Math.pow(heightToSpeaker, 2) + Math.pow(depthToWall, 2)) -
        (2 * heightToSpeaker * depthToWall * Math.cos(wallAngle))
    );

    // Using law of sines, find the angle from the limelight face to the speaker center in degrees
    double angle = Math.asin(
        heightToSpeaker * (Math.sin(wallAngle) / lengthToSpeaker)
    ) * (180 / Math.PI);

    // Inverse the angle for use with the arm
    return Constants.ShooterConstants.limelightDetectAngle - angle;
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

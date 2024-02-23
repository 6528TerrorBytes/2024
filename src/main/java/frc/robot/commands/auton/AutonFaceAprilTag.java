// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AutonFaceAprilTag extends Command {
  private final DriveSubsystem m_driveSubsystem;

  /** Creates a new AutonFaceAprilTag. */
  public AutonFaceAprilTag(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed;
    // If see a limelight,
    if (LimelightHelpers.getTV("limelight")) {
      // Tx is the offset from the center of the camera (2d not 3d) 
      double tx = LimelightHelpers.getTX("limelight"); 
      rotationSpeed = -tx / 30;

      // Clamp
      // if (rotationSpeed > DriveSubsystem.speedMultiplier) {
      //   rotationSpeed = DriveSubsystem.speedMultiplier;
      // } else if (rotationSpeed < -DriveSubsystem.speedMultiplier) {
      //   rotationSpeed = -DriveSubsystem.speedMultiplier;
      // }

      if (rotationSpeed > 0.5) {
        rotationSpeed = 0.5;
      } else if (rotationSpeed < -0.5) {
        rotationSpeed = -0.5;
      }

      rotationSpeed *= 0.5;
    } else {
      rotationSpeed = 0.2;
    }

    m_driveSubsystem.drive(0, 0, rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

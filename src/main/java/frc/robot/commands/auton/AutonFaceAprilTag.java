// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.DriveSubsystem;

public class AutonFaceAprilTag extends Command {
  private final DriveSubsystem m_driveSubsystem;

  private double tagTX;

  /** Creates a new AutonFaceAprilTag. */
  public AutonFaceAprilTag(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationSpeed;

    // If see a limelight and correct speaker ID
    if (Utility.aprilTagInView()) {
      tagTX = Utility.getTX();
      rotationSpeed = Utility.calcSpeedFaceTag(tagTX);
    } else {
      rotationSpeed = 0;
    }

    m_driveSubsystem.drive(0, 0, rotationSpeed, true, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(tagTX) <= Constants.AutonConstants.aprilTagHorizontalEndRange;
  }
}

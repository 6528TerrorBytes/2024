// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utility;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopFaceAprilTag extends Command {
  public static boolean disable = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.overrideRotation = Utility.aprilTagInView();
    if (!DriveSubsystem.overrideRotation) { return; } // Check for AprilTag in view
    
    DriveSubsystem.newRotation = Utility.calcSpeedFaceTag(Utility.getTX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.overrideRotation = false;
    disable = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return disable;
  }
}

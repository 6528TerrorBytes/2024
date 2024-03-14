// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterTilt;

public class OverrideShooterDisable extends Command {
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterTilt.overrideCheck = false;
    System.out.println("overrideCheck false");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterTilt.overrideCheck = true;
    System.out.println("overrideCheck true");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

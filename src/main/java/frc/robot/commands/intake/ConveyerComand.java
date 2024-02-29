// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ConveyerSubsystem;

public class ConveyerComand extends Command {
  private final ConveyerSubsystem m_conveyerSubsystem;

  private double m_speed;

  /** Creates a new ConveyerComand. */
  public ConveyerComand(ConveyerSubsystem conveyerSubsystem, double speed) {
    m_conveyerSubsystem = conveyerSubsystem;
    m_speed = speed;
    addRequirements(m_conveyerSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_conveyerSubsystem.setSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

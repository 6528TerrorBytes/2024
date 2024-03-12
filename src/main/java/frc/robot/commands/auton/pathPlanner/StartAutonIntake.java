// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.pathPlanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Starts intake for auton and the command immediately ends.
 * Call in unison with the StopNoteCommand
 */
public class StartAutonIntake extends Command {
  private final ConveyerSubsystem m_conveyerSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  public StartAutonIntake(ConveyerSubsystem conveyerSubsystem, IntakeSubsystem intakeSubsystem) {
    m_conveyerSubsystem = conveyerSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_conveyerSubsystem, m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_conveyerSubsystem.setSpeed(1);
    m_intakeSubsystem.setSpeed(1);
  }

  @Override
  public boolean isFinished() { return true; }
}

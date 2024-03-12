// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.pathPlanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DetectNote;
import frc.robot.subsystems.IntakeSubsystem;

public class EndAutonIntake extends Command {
  private final DetectNote m_detectNote;
  private final ConveyerSubsystem m_conveyerSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  private double m_endTime;

  public EndAutonIntake(ConveyerSubsystem conveyerSubsystem, IntakeSubsystem intakeSubsystem, DetectNote detectNote) {
    m_conveyerSubsystem = conveyerSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_detectNote = detectNote;
    addRequirements(m_conveyerSubsystem, m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    m_endTime = Utility.getTime() + Constants.AutonConstants.intakeRunTime;
  }

  @Override
  public void end(boolean interrupted) {
    m_conveyerSubsystem.stop();
    m_intakeSubsystem.stop();
  }

  // Stops either when the bot has a ring or the wait time is up
  @Override
  public boolean isFinished() {
    return (Utility.getTime() >= m_endTime) || m_detectNote.activated();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DetectNote;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final DetectNote m_detectNote;

  private double m_speed;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, DetectNote detectNote, double speed) {
    m_intakeSubsystem = intakeSubsystem;
    m_detectNote = detectNote;
    m_speed = speed;
    addRequirements(m_intakeSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_speed < 0 || !m_detectNote.activated()) {
      m_intakeSubsystem.setSpeed(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_speed < 0) {
      return false;
    }

    return m_detectNote.activated();
  }
}

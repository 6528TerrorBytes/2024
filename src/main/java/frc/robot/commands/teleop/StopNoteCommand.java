// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StopNote;

public class StopNoteCommand extends Command {
  private final StopNote m_stopNote;

  private final boolean goToEnd;

  /** Creates a new HangerArmCommand. */
  public StopNoteCommand(StopNote hangerArm, boolean goToEnd) {
    m_stopNote = hangerArm;
    addRequirements(m_stopNote);
    this.goToEnd = goToEnd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stopNote.setTolerance(10);

    if (goToEnd) { m_stopNote.setExtended(); }
    else { m_stopNote.setClosed(); }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_stopNote.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_stopNote.atGoal();
  }
}

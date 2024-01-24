// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ChangeSpeed;

public class IncreaseSpeed extends Command {
  private ChangeSpeed m_changeSpeed;

  /** Creates a new IncreaseSpeed. */
  public IncreaseSpeed(ChangeSpeed changeSpeed) {
    m_changeSpeed = changeSpeed;
    addRequirements(m_changeSpeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_changeSpeed.setMax();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_changeSpeed.setNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

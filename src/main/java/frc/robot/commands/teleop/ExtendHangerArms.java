// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangerArm;

public class ExtendHangerArms extends Command {
  private final HangerArm m_hangerArm;
  private final double m_speed = 0.8;
  private boolean m_reversed;

  /** Creates a new ExtendHangerArms. */
  public ExtendHangerArms(HangerArm hangerArm, boolean reversed) {
    m_hangerArm = hangerArm;
    m_reversed = reversed;
    addRequirements(hangerArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (m_reversed) {
      m_hangerArm.setSpeed(m_speed);
    } else {
      m_hangerArm.setSpeed(-m_speed);
    }
    m_hangerArm.checkBoundaries();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hangerArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AmpFlap;

public class AmpFlapCommand extends Command {
  private final AmpFlap m_ampFlap;
  private final boolean m_isDown;

  public AmpFlapCommand(AmpFlap ampFlap, boolean isDown) {
    m_ampFlap = ampFlap;
    m_isDown = isDown;
    addRequirements(m_ampFlap);
  }

  @Override
  public void initialize() {
    if (m_isDown) { m_ampFlap.setDown(); }
    else { m_ampFlap.setUp(); }
  }

  @Override
  public boolean isFinished() {
    return m_ampFlap.atGoal();
  }
}

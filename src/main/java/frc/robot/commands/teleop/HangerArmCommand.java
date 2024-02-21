// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangerArm;

public class HangerArmCommand extends Command {
  private final HangerArm m_hangerArm;

  private final boolean goToEnd;

  /** Creates a new HangerArmCommand. */
  public HangerArmCommand(HangerArm hangerArm, boolean goToEnd) {
    m_hangerArm = hangerArm;
    addRequirements(m_hangerArm);
    this.goToEnd = goToEnd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hangerArm.setTolerance(10);

    if (goToEnd) { m_hangerArm.setExtended(); }
    else { m_hangerArm.setClosed(); }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hangerArm.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hangerArm.atGoal();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.HangerArm;

public class LimitSwitchDetector extends Command {
  DigitalInput leftLimitSwitch = new DigitalInput(1);
  DigitalInput rightLimitSwitch = new DigitalInput(0);

  // private HangerArm m_hangerArm;

  /** Creates a new LimitSwitchDetector. */
  public LimitSwitchDetector() {
    // m_hangerArm = hangerArm;
    // addRequirements(m_hangerArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("left limit switch ", leftLimitSwitch.get());
    SmartDashboard.putBoolean("right limit switch ", rightLimitSwitch.get());

    if (!leftLimitSwitch.get()) {
      System.out.println("left switch pressed");
      ExtendHangerArms.m_leftStopped = true;
    }

    if (!rightLimitSwitch.get()) {
      System.out.println("right switch pressed");
    }

    ExtendHangerArms.m_leftStopped = !leftLimitSwitch.get();
    ExtendHangerArms.m_rightStopped = !rightLimitSwitch.get();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

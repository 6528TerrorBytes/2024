// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DetectNote extends SubsystemBase {
  private final DigitalInput m_limitSwitch = new DigitalInput(Constants.LimitSwitches.detectNote);

  public boolean activated() {
    return !m_limitSwitch.get();
  }

  // Make a command that requires this subsystem and the Blinkin subsystem that sets its color
}

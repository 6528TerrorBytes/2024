// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility;

public class Blinkin extends SubsystemBase {
  private Spark m_blinkin = new Spark(Constants.BlinkinConstants.id);
  private double m_currentColor = 0;

  /** Creates a new Blinkin. */
  public Blinkin() {}

  public void resetToTeamColor(
    double blueTeamColor,
    double redTeamColor
  ) {
    if (Utility.teamColorIsRed()) {
      setColor(redTeamColor);
    } else {
      setColor(blueTeamColor);
    }
  }

  public void setColor(double color) {
    if (m_currentColor != color) {
      m_currentColor = color;
      m_blinkin.set(color);
    }
  }

  public void getColor() {
    return m_currentColor;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.DetectNote;

public class NoteBlinkinColor extends Command {
  private final Blinkin m_blinkin;
  private final DetectNote m_detectNote;

  /** Creates a new NoteBlinkinColor. */
  public NoteBlinkinColor(Blinkin blinkin, DetectNote detectNote) {
    m_blinkin = blinkin;
    m_detectNote = detectNote;
    addRequirements(m_blinkin);
  }

  @Override
  public void execute() {
    double secondsLeftInPeriod = Utility.getMatchTime();
    
    if (m_detectNote.activated()) {
      m_blinkin.setColor(Constants.BlinkinConstants.green);
    } else {
      m_blinkin.resetToTeamColor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

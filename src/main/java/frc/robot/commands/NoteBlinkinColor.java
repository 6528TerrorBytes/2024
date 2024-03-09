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

  private boolean m_isTeleop;

  private final double timeToWarn = 20; // Warn at 20 seconds left
  private final double warnEnd = 18; // Time when the flashing ends

  /** Creates a new NoteBlinkinColor. */
  public NoteBlinkinColor(Blinkin blinkin, DetectNote detectNote) {
    m_blinkin = blinkin;
    m_detectNote = detectNote;
    addRequirements(m_blinkin);
  }

  @Override
  public void execute() {
    double secondsLeftInPeriod = Utility.getMatchTime();

    // Between the time where the flash occurs
    if (m_isTeleop && (warnEnd < secondsLeftInPeriod) && (secondsLeftInPeriod < timeToWarn)) {
      m_blinkin.resetToTeamColor( // Set to strobe colors
        Constants.BlinkinConstants.strobeBlue,
        Constants.BlinkinConstants.strobeRed
      );
      return;
    }
    
    // Sets to green if it has a note, otherwise to the team color
    if (m_detectNote.activated()) {
      m_blinkin.setColor(Constants.BlinkinConstants.green);
    } else {
      m_blinkin.resetToTeamColor(
        Constants.BlinkinConstants.blue,
        Constants.BlinkinConstants.red
      );
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Command runs indefinitely
  }

  public void setTeleop(boolean isTeleop) {
    m_isTeleop = isTeleop;
  }
}

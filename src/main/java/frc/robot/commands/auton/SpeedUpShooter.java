// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.ShooterSubsystem;

// Speeds up the shooter. Command ends when it has fully sped up.
public class SpeedUpShooter extends Command {
  private final ShooterSubsystem m_shooterSubsystem;

  private double m_timeToFinish;

  private double m_speed;

  private double m_speedUpSecs;
  private boolean m_endWhenInterrupted;

  /** Creates a new SpeedUpShooter. */
  public SpeedUpShooter(ShooterSubsystem shooterSubsystem, double speed, double speedUpSecs) {
  m_shooterSubsystem = shooterSubsystem;
    m_speed = speed;
    m_speedUpSecs = speedUpSecs;
    m_endWhenInterrupted = true;
  }

  public SpeedUpShooter(ShooterSubsystem shooterSubsystem, double speed, boolean endWhenInterrupted) {
    m_shooterSubsystem = shooterSubsystem;
    m_speed = speed;
    m_speedUpSecs = -1;
    m_endWhenInterrupted = endWhenInterrupted;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_speedUpSecs >= 0) {
      m_shooterSubsystem.setSpeed(m_speed);
      m_shooterSubsystem.setForward();
      m_timeToFinish = Utility.getTime() + m_speedUpSecs;
    } else if (!m_endWhenInterrupted && m_shooterSubsystem.getSpeed() != 0) {
      m_shooterSubsystem.stop();
    } else {
      m_shooterSubsystem.setSpeed(m_speed);
      m_shooterSubsystem.setForward();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted && m_endWhenInterrupted) { // Joystick button let go of
      m_shooterSubsystem.stop();
      System.out.println("Ended");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_speedUpSecs > 0 && (Utility.getTime() >= m_timeToFinish);
  }
}

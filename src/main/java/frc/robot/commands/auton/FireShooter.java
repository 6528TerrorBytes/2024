// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utility;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Run the SpeedUpShooter before this!
public class FireShooter extends Command {
  private final ConveyerSubsystem m_conveyerSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  
  private double m_timeToFinish;

  private double m_fireTimeSecs;

  /** Creates a new FireShooter. */
  public FireShooter(ConveyerSubsystem conveyerSubsystem, ShooterSubsystem shooterSubsystem, double fireTimeSecs) {
    m_conveyerSubsystem = conveyerSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_fireTimeSecs = fireTimeSecs;
    addRequirements(m_conveyerSubsystem, m_shooterSubsystem); 
  }

  @Override
  public void initialize() {
    m_conveyerSubsystem.setSpeed(1);
    m_timeToFinish = Utility.getTime() + m_fireTimeSecs;
  }

  @Override
  public void end(boolean interrupted) {
    m_conveyerSubsystem.stop();
    m_shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Utility.getTime() >= m_timeToFinish;
  }
}

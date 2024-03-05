// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DetectNote;

public class ConveyerComand extends Command {
  private final ConveyerSubsystem m_conveyerSubsystem;
  private final DetectNote m_detectNote;

  private double m_speed;
  private boolean m_stopWhenDetected; // Whether or not to stop when a ring has been detected

  /** Creates a new ConveyerComand. */
  public ConveyerComand(ConveyerSubsystem conveyerSubsystem, DetectNote detectNote, double speed, boolean stopWhenDetected) {
    m_conveyerSubsystem = conveyerSubsystem;
    m_detectNote = detectNote;
    m_speed = speed;
    m_stopWhenDetected = stopWhenDetected;

    addRequirements(m_conveyerSubsystem); // never require the DetectNote class
  }

  @Override
  public void initialize() {
    if (!m_stopWhenDetected || (m_stopWhenDetected && !m_detectNote.activated())) {
      m_conveyerSubsystem.setSpeed(m_speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_stopWhenDetected && m_detectNote.activated()) {
    //   m_conveyerSubsystem.stop();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_stopWhenDetected) { return false; } // Only finishes when you stop pressing the joystick button
    else { 
      // return m_detectNote.activated();

      if (m_detectNote.activated()) {
        System.out.println("ended conveyer");
        return true;
      } else {
        return false;
      }
    } // Ends when a note has been detected
  }
}

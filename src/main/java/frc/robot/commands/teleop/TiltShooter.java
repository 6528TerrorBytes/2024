// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterTiltSubsystem;

public class TiltShooter extends Command {
  private final ShooterTiltSubsystem m_shooterTiltSubsystem;

  /** Creates a new TiltShooter. */
  public TiltShooter(ShooterTiltSubsystem shooterTiltSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterTiltSubsystem = shooterTiltSubsystem;
    addRequirements(m_shooterTiltSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterTiltSubsystem.setGoal(Math.PI); // Sets the shooter to tilt 180 degrees
    m_shooterTiltSubsystem.setTolerance(0.2, 0.4); // in radians
    m_shooterTiltSubsystem.enable(); // Enable the subsystem to move the motor
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterTiltSubsystem.disable(); // Disable the subsystem
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterTiltSubsystem.atGoal();
  }
}

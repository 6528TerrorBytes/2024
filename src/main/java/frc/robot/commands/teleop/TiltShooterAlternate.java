// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterTilt;

public class TiltShooterAlternate extends Command {
  private final ShooterTilt m_shooterTilt;

  private final double angle = Math.PI;

  /** Creates a new TiltShooterAlternate. */
  public TiltShooterAlternate(ShooterTilt shooterTilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterTilt = shooterTilt;
    addRequirements(m_shooterTilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterTilt.setAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterTilt.angleBetween(angle - 0.1, angle + 0.1);
  }
}

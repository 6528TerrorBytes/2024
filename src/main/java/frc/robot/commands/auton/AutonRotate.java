// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utility;
import frc.robot.subsystems.DriveSubsystem;

public class AutonRotate extends Command {
  private final DriveSubsystem m_driveSubsystem;

  private double m_angleGoal;
  private double m_diff;

  private final double tolerance = 5;

  /** Creates a new AutonRotate. */
  public AutonRotate(DriveSubsystem driveSubsystem, double gyroAngle) {
    m_driveSubsystem = driveSubsystem;
    m_angleGoal = gyroAngle;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_diff = m_driveSubsystem.getRawAngle() - m_angleGoal;
    double speed = Utility.calcSpeedFaceTag(m_diff);
    
    m_driveSubsystem.drive(0, 0, speed, true, true, false);
  }

  // Called once the command ends or is interrupted.
  // Call of Duty
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_diff) < tolerance;
  }
}

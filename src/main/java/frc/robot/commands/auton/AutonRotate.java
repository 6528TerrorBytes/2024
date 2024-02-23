// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

public class AutonRotate extends Command {
  private final DriveSubsystem m_driveSubsystem;

  private int angleGoal;

  /** Creates a new AutonRotate. */
  public AutonRotate(DriveSubsystem driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleGoal = (int)(m_driveSubsystem.getRawAngle() + 45); // Moving 45 degrees
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(0, 0, -0.5, true, true);
  }

  // Called once the command ends or is interrupted.
  // Call of Duty
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (int)(m_driveSubsystem.getRawAngle()) >= angleGoal;
  }
}

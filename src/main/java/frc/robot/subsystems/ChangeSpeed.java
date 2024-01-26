// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.DriveSubsystem;

public class ChangeSpeed extends SubsystemBase {
  private final double maxSpeed = 1.0;
  private final double neutralSpeed = 0.5;
  private final double lowerSpeed = 0.2;

  /** Creates a new ChangeSpeed. */
  public ChangeSpeed() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMax() {
    DriveSubsystem.speedMultiplier = maxSpeed;
  }

  public void setNeutral() {
    DriveSubsystem.speedMultiplier = neutralSpeed;
  }

  public void setLower() {
    DriveSubsystem.speedMultiplier = lowerSpeed;
  }
}

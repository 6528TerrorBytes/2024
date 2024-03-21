// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public final CANSparkMax shooterLeft =  new CANSparkMax(Constants.MotorIDs.shooterLeft,  MotorType.kBrushless);
  public final CANSparkMax shooterRight = new CANSparkMax(Constants.MotorIDs.shooterRight, MotorType.kBrushless);

  private double m_speed = 0;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    m_speed = speed;
  }

  public double getSpeed() {
    return m_speed;
  }

  public void setForward() {
    shooterLeft.set(-m_speed);
    shooterRight.set(m_speed);
  }

  public void setReverse() {
    shooterLeft.set(m_speed);
    shooterRight.set(-m_speed);
  }

  public void stop() {
    setSpeed(0);
    setForward();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerArm extends SubsystemBase {
  private final CANSparkMax m_leftArmMotor = new CANSparkMax(Constants.MotorIDs.hangerArmLeft, MotorType.kBrushless);
  private final CANSparkMax m_rightArmMotor = new CANSparkMax(Constants.MotorIDs.hangerArmRight, MotorType.kBrushless);

  private final RelativeEncoder m_leftRelativeEncoder = m_leftArmMotor.getEncoder();
  private final RelativeEncoder m_rightRelativeEncoder = m_rightArmMotor.getEncoder();

  /** Creates a new HangerArm. */
  public HangerArm() {
    m_leftArmMotor.setIdleMode(IdleMode.kBrake);
    m_rightArmMotor.setIdleMode(IdleMode.kBrake);

    m_leftArmMotor.burnFlash();
    m_rightArmMotor.burnFlash();

    zeroEncoders(); // ---- TEMPORARY ----
  }

  public void setSpeed(double speed) {
    m_leftArmMotor.set(-speed);
    m_rightArmMotor.set(speed);
  }

  public void setSpeedLeft(double speed) {
    m_leftArmMotor.set(-speed);
  }

  public void setSpeedRight(double speed) {
    m_rightArmMotor.set(speed);
  }

  public void zeroEncoders() {
    m_leftRelativeEncoder.setPosition(0);
    m_rightRelativeEncoder.setPosition(0);
  }

  public void zeroEncoderLeft() {
    m_leftRelativeEncoder.setPosition(0);
  }

  public void zeroEncoderRight() {
    m_leftRelativeEncoder.setPosition(0);
  }

  public void getEncoderLeft() {
    m_leftRelativeEncoder.getPosition();
  }

  public void getEncoderRight() {
    m_rightRelativeEncoder.getPosition();
  }

  public void stop() {
    m_leftArmMotor.set(0);
    m_rightArmMotor.set(0);
  }

  public void stopLeft() {
    m_leftArmMotor.set(0);
  }

  public void stopRight() {
    m_rightArmMotor.set(0);
  }

  public double getSpeedLeft() {
    return m_leftArmMotor.get();
  }

  public double getSpeedRight() {
    return m_rightArmMotor.get();
  }
}

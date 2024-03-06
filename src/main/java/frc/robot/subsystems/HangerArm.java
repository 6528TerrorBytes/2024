// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerArm extends SubsystemBase {
  private final CANSparkMax m_leftArmMotor = new CANSparkMax(Constants.MotorIDs.hangerArmLeft, MotorType.kBrushless);
  private final CANSparkMax m_rightArmMotor = new CANSparkMax(Constants.MotorIDs.hangerArmRight, MotorType.kBrushless);

  private final RelativeEncoder m_leftRelativeEncoder = m_leftArmMotor.getEncoder();
  private final RelativeEncoder m_rightRelativeEncoder = m_rightArmMotor.getEncoder();

  private final DigitalInput m_rightLimitSwitch = new DigitalInput(Constants.LimitSwitches.rightHangerArm);
  private final DigitalInput m_leftLimitSwitch = new DigitalInput(Constants.LimitSwitches.leftHangerArm);

  /** Creates a new HangerArm. */
  public HangerArm() {
    m_leftArmMotor.setIdleMode(IdleMode.kBrake);
    m_rightArmMotor.setIdleMode(IdleMode.kBrake);

    m_leftArmMotor.burnFlash();
    m_rightArmMotor.burnFlash();

    zeroEncoders(); // ---- TEMPORARY ----
  }

  // Call periodically to check limit switches and the encoders
  // Only stops the motors if they're moving downward
  // Also checks upper bounds for the arms
  public void checkBoundaries() {
    if (getSpeedLeft() > 0) { // When moving down
      if (!m_leftLimitSwitch.get()) {
        stopLeft();
        zeroEncoderLeft();
      }
    } else if (getEncoderLeft() < -Constants.HangerArmConstants.upperEncoderLimit) { // When moving up
      stopLeft();
    }

    if (getSpeedRight() < 0) { // When moving down
      if (!m_rightLimitSwitch.get()) {
        stopRight();
        zeroEncoderRight();
      }
    } else if (getEncoderRight() > Constants.HangerArmConstants.upperEncoderLimit) {
      stopRight();
    }
  }

  public void outputLimitSwitches() {
    SmartDashboard.putBoolean("leftLimitSwitch", !m_leftLimitSwitch.get());
    SmartDashboard.putBoolean("rightLimitSwitch", !m_rightLimitSwitch.get());
  }

  // Set speeds
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

  // Encoders
  public void zeroEncoders() {
    m_leftRelativeEncoder.setPosition(0);
    m_rightRelativeEncoder.setPosition(0);
  }

  public void zeroEncoderLeft() {
    m_leftRelativeEncoder.setPosition(0);
  }

  public void zeroEncoderRight() {
    m_rightRelativeEncoder.setPosition(0);
  }

  public double getEncoderLeft() {
    return m_leftRelativeEncoder.getPosition();
  }

  public double getEncoderRight() {
    return m_rightRelativeEncoder.getPosition();
  }

  // Other motor functions
  public void stop() {
    setSpeed(0);
  }

  public void stopLeft() {
    setSpeedLeft(0);
  }

  public void stopRight() {
    setSpeedRight(0);
  }

  public double getSpeedLeft() {
    return m_leftArmMotor.get();
  }

  public double getSpeedRight() {
    return m_rightArmMotor.get();
  }
}

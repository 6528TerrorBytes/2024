// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    zeroEncoders(); // ---- TEMPORARY ----
  }

  public void setSpeed(double speed) {
    m_leftArmMotor.set(speed);
    m_rightArmMotor.set(speed);

    System.out.println("left speed " + Double.toString(m_leftArmMotor.get()));
    System.out.println("right speed " + Double.toString(m_rightArmMotor.get()));

    System.out.println("left encoder " + Double.toString(m_leftRelativeEncoder.getPosition()));
    System.out.println("right encoder " + Double.toString(m_rightRelativeEncoder.getPosition()));
  }

  public void zeroEncoders() {
    m_leftRelativeEncoder.setPosition(0);
    m_rightRelativeEncoder.setPosition(0);
  }

  public void stop() {
    m_leftArmMotor.set(0);
    m_rightArmMotor.set(0);
  }
}

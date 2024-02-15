// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

/* May not be used in favor of the ShooterTiltSubsystem */

public class ShooterTilt extends SubsystemBase {
  private final CANSparkMax tiltMotor = new CANSparkMax(Constants.MotorIDs.shooterTilt, MotorType.kBrushless);
  private final AbsoluteEncoder tiltEncoder = tiltMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController pidController = tiltMotor.getPIDController();

  /** Creates a new ShooterTilt. */
  public ShooterTilt() {
    // Reset before configuring
    tiltMotor.restoreFactoryDefaults();

    // Set the encoder for the PID Controller
    pidController.setFeedbackDevice(tiltEncoder);

    pidController.setP(Constants.ShooterConstants.tiltP);
    pidController.setI(Constants.ShooterConstants.tiltI);
    pidController.setD(Constants.ShooterConstants.tiltD);
    pidController.setFF(0); // Feedforward gains (?)
    pidController.setOutputRange(-1, 1);

    // Setup encoder conversions to use radians and radians per second
    tiltEncoder.setPositionConversionFactor(Constants.ShooterConstants.toRadians);
    tiltEncoder.setVelocityConversionFactor(Constants.ShooterConstants.toRadiansPerSec);
    
    tiltMotor.setIdleMode(IdleMode.kBrake);
    tiltMotor.setSmartCurrentLimit(20);

    tiltMotor.burnFlash(); // Save settings
  }

  // Moves to the given angle (in radians)
  public void setAngle(double angle) {
    pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
  }

  public double getAngle() {
    return tiltEncoder.getPosition();
  }
}

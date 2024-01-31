// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public final CANSparkMax intakeMotor = new CANSparkMax(Constants.MotorIDs.intake, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    stop();
  }

  public void start() {
    intakeMotor.set(-1);
  }

  public void reverse() {
    intakeMotor.set(0.5);
  }

  public void slow() {
    intakeMotor.set(-0.2);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

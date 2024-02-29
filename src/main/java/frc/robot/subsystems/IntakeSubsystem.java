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
  
  public void setSpeed(double speed) {
    intakeMotor.set(-speed);
  }

  public void stop() {
    intakeMotor.set(0);
  }
}

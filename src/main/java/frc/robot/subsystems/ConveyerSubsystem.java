// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyerSubsystem extends SubsystemBase {
  public final CANSparkMax conveyerLeft =  new CANSparkMax(Constants.MotorIDs.conveyerLeft,  MotorType.kBrushless);
  public final CANSparkMax conveyerRight = new CANSparkMax(Constants.MotorIDs.conveyerRight, MotorType.kBrushless);

  private final double speed = 0.9;

  /** Creates a new ConveyerSubsystem. */
  public ConveyerSubsystem() {
    stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setForward() {
    conveyerLeft.set(-speed);
    conveyerRight.set(speed);
  }

  public void setReverse() {
    conveyerLeft.set(speed);
    conveyerRight.set(-speed);
  }

  public void stop() {
    conveyerLeft.set(0);
    conveyerRight.set(0);
  }
}

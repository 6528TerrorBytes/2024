// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;

public class StopNote extends AngleMotor {
  public StopNote() {
    super(
      Constants.MotorIDs.stopNote,
      Constants.StopNoteConstants.minAngle,
      Constants.StopNoteConstants.maxAngle
    );

    getController().setP(Constants.StopNoteConstants.p, 0);
    getController().setI(Constants.StopNoteConstants.i, 0);
    getController().setD(Constants.StopNoteConstants.d, 0);
    getController().setFF(0, 0); // Feedforward gains (?)
    setSpeed(Constants.StopNoteConstants.speed);
    
    getMotor().setInverted(true);
    getMotor().setIdleMode(IdleMode.kBrake);
    getMotor().setSmartCurrentLimit(10); // Just torque (strength), not speed

    burnFlash(); // Save settings

    setTolerance(Constants.StopNoteConstants.tolerance);
  }

  public void setClosed() {
    setGoal(Constants.StopNoteConstants.closedAngle);
  }
  
  public void setOpen() {
    setGoal(Constants.StopNoteConstants.openAngle);
  }
}
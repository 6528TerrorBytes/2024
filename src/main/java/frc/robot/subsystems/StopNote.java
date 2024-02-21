// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;

public class StopNote extends AngleMotor {
    public StopNote() {
        super(
            Constants.MotorIDs.hangerArm,
            Constants.StopNoteConstants.minAngle,
            Constants.StopNoteConstants.maxAngle
        );

        getController().setP(1, 0);
        getController().setI(0, 0);
        getController().setD(0, 0);
        getController().setFF(0, 0); // Feedforward gains (?)
        getController().setOutputRange(-0.25, 0.25); // Control speed -1 to 1 (motor output)
        
        getMotor().setInverted(true);
        getMotor().setIdleMode(IdleMode.kBrake);
        getMotor().setSmartCurrentLimit(10); // Just torque (strength), not speed

        burnFlash(); // Save settings
    }

    public void setClosed() {
        setGoal(Constants.StopNoteConstants.closedAngle);
    }
    
    public void setOpen() {
        setGoal(Constants.StopNoteConstants.openAngle);
    }
}
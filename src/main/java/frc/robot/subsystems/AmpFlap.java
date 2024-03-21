// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class AmpFlap extends AngleMotor {
  public AmpFlap() {
    super(
      Constants.MotorIDs.ampFlap,
      Constants.AmpFlapConstants.minAngle, 
      Constants.AmpFlapConstants.maxAngle, 
      true, (4096 * 25) / 360, (4096 * 25)
    );

    getController().setP(Constants.AmpFlapConstants.p, 0);
    getController().setI(Constants.AmpFlapConstants.i, 0);
    getController().setD(Constants.AmpFlapConstants.d, 0);
    getController().setFF(0, 0); // Feedforward gains (?)
    setSpeed(Constants.AmpFlapConstants.speed);
    
    getMotor().setInverted(false);
    getMotor().setIdleMode(IdleMode.kBrake);
    getMotor().setSmartCurrentLimit(10); // Just torque (strength), not speed

    burnFlash(); // Save settings

    setTolerance(Constants.AmpFlapConstants.tolerance);
  }

  public void setDown() {
    setGoal(Constants.AmpFlapConstants.downAngle);
  }

  public void setUp() {
    setGoal(Constants.AmpFlapConstants.upAngle);
  }

  public void outputEncoder() {
    SmartDashboard.putNumber("Amp flap encoder angle", getAngle());
  }
}

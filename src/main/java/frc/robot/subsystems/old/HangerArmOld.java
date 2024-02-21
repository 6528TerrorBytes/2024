// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.old;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerArmOld extends SubsystemBase {
  private final CANSparkMax tiltMotor = new CANSparkMax(Constants.MotorIDs.hangerArm, MotorType.kBrushless);
  private final AbsoluteEncoder tiltEncoder = tiltMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController pidController = tiltMotor.getPIDController();
  
  private double angleGoal = 0;
  private double tolerance = 0;

  /** Creates a new HangerArmOld. */
  public HangerArmOld() {
    // Set the encoder for the PID Controller
    pidController.setFeedbackDevice(tiltEncoder);

    pidController.setP(1, 0);
    pidController.setI(0, 0);
    pidController.setD(0, 0);
    pidController.setFF(0, 0);
    pidController.setOutputRange(-0.15, 0.15); // Control speed -1 to 1 (motor output)

    pidController.setPositionPIDWrappingEnabled(true);
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingMaxInput(360);
    
    // Setup encoder conversions to use radians and radians per second
    tiltEncoder.setPositionConversionFactor(Constants.toDegrees);
    tiltEncoder.setVelocityConversionFactor(Constants.toDegreesPerSec);
    
    tiltMotor.setInverted(true);
    tiltMotor.setIdleMode(IdleMode.kBrake);
    tiltMotor.setSmartCurrentLimit(30); // Just torque (strength), not speed

    tiltMotor.burnFlash(); // Save settings
  }

  // Moves to the given angle (in degrees)
  public void setGoal(double angle) {
    angleGoal = angle;

    // Clamp the angle goal between the angle limits
    if (angleGoal < Constants.HangerConstants.minAngle) {
      angleGoal = Constants.HangerConstants.minAngle;
    } else if (angleGoal > Constants.HangerConstants.maxAngle) {
      angleGoal = Constants.HangerConstants.maxAngle;
    }

    // Set the PID controller to move to the angle
    pidController.setReference(angleGoal, CANSparkMax.ControlType.kPosition);
  }

  public void setTolerance(double angleOffset) {
    tolerance = angleOffset;
  }

  public double getAngle() {
    double angle = tiltEncoder.getPosition();
    SmartDashboard.putNumber("Hanger tilt encoder angle ", angle);
    return angle;
  }

  public void setExtended() {
    setGoal(Constants.HangerConstants.extendedAngle);
  }

  public void setClosed() {
    setGoal(Constants.HangerConstants.closedAngle);
  }

  /**
   * Tests if the angle of the Hanger has reached between + or - tolerance
   * from the angle goal. Returns true if so.
   */
  public boolean atGoal() {
    double angle = getAngle();
    SmartDashboard.putNumber("Hanger Angle Goal: ", angleGoal);
    SmartDashboard.putNumber("Hanger Tolerance: ", tolerance);

    boolean success = ((angleGoal - tolerance) <= angle) && (angle <= (angleGoal + tolerance));
    SmartDashboard.putBoolean("Hanger reached goal: ", success);

    return success;
  }

  public void disable() {
    tiltMotor.set(0);
  }
}

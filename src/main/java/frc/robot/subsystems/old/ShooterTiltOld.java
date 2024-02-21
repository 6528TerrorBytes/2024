// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.old;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

/* May not be used in favor of the ShooterTiltOldSubsystem */

public class ShooterTiltOld extends SubsystemBase {
  private final CANSparkMax tiltMotor = new CANSparkMax(Constants.MotorIDs.shooterTilt, MotorType.kBrushless);
  private final AbsoluteEncoder tiltEncoder = tiltMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkPIDController pidController = tiltMotor.getPIDController();

  private double angleGoal = 0;
  private double tolerance = 0;

  /** Creates a new ShooterTiltOld. */
  public ShooterTiltOld() {
    // Set the encoder for the PID Controller
    pidController.setFeedbackDevice(tiltEncoder);

    pidController.setP(Constants.ShooterConstants.tiltP, 0);
    pidController.setI(Constants.ShooterConstants.tiltI, 0);
    pidController.setD(Constants.ShooterConstants.tiltD, 0);
    pidController.setFF(0, 0); // Feedforward gains (?)
    pidController.setOutputRange(-0.25, 0.25); // Control speed -1 to 1 (motor output)

    // pidController.setSmartMotionMaxAccel(1, 0);
    // pidController.setSmartMotionMaxVelocity(5, 0);

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
    angleGoal = angle + Constants.ShooterConstants.angleOffset;

    // Clamp the angle goal between the angle limits
    if (angleGoal < Constants.ShooterConstants.minAngle) {
      angleGoal = Constants.ShooterConstants.minAngle;
    } else if (angleGoal > Constants.ShooterConstants.maxAngle) {
      angleGoal = Constants.ShooterConstants.maxAngle;
    }

    // Set the PID controller to move to the angle
    pidController.setReference(angleGoal, CANSparkMax.ControlType.kPosition);
  }

  public void setTolerance(double angleOffset) {
    tolerance = angleOffset;
  }

  public double getAngle() {
    double angle = tiltEncoder.getPosition();
    SmartDashboard.putNumber("Shooter tilt encoder angle ", angle);
    return angle;
  }

  /**
   * Tests if the angle of the shooter has reached between + or - tolerance
   * from the angle goal. Returns true if so.
   */
  public boolean atGoal() {
    double angle = getAngle();
    SmartDashboard.putNumber("Shooter tilt Angle Goal: ", angleGoal);
    SmartDashboard.putNumber("Shooter tolerance: ", tolerance);

    boolean success = ((angleGoal - tolerance) <= angle) && (angle <= (angleGoal + tolerance));
    SmartDashboard.putBoolean("Shooter Reached goal: ", success);

    return success;
  }

  public void disable() {
    tiltMotor.set(0);
  }
}

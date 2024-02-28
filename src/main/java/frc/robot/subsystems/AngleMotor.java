// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AngleMotor extends SubsystemBase {
  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;
  private final SparkPIDController pidController;

  private double angleGoal = 0;
  private double tolerance = 10;
  private double startingOffset = 0;

  private double minAngle;
  private double maxAngle;

  /** Creates a new AngleMotor. */
  public AngleMotor(int motorID, double minAngle, double maxAngle) {
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    pidController = motor.getPIDController();

    pidController.setFeedbackDevice(encoder);

    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
        
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingMaxInput(360);
    pidController.setPositionPIDWrappingEnabled(true);

    // Setup encoder conversions to use radians and radians per second
    encoder.setPositionConversionFactor(Constants.toDegrees);
    encoder.setVelocityConversionFactor(Constants.toDegreesPerSec);
  }
  
  public void burnFlash() {
    motor.burnFlash();
  }

  public void setStartingOffset(double offset) {
    startingOffset = offset;
  }

  // Need to burn flash after running this function
  public void setSpeed(double speed) {
    // Control speed from -1 to 1 (min/max motor speed)
    pidController.setOutputRange(-speed, speed);
  }

  // Moves to the given angle (in degrees)
  public void setGoal(double angle) {
    angleGoal = angle + startingOffset;

    // Clamp the angle goal between the angle limits
    if (angleGoal < minAngle) {
      angleGoal = minAngle;
    } else if (angleGoal > maxAngle) {
      angleGoal = maxAngle;
    }

    // Set the PID controller to move to the angle
    pidController.setReference(angleGoal, CANSparkMax.ControlType.kPosition);
  }

  public void setTolerance(double angleOffset) {
    tolerance = angleOffset;
  }

  public double getAngle() {
    double angle = encoder.getPosition();
    return angle;
  }
  
  /**
   * Tests if the angle of the motor has reached between + or - tolerance
   * from the angle goal. Returns true if so.
   */
  public boolean atGoal() {
    double angle = getAngle();
    return ((angleGoal - tolerance) <= angle) && (angle <= (angleGoal + tolerance));
  }

  public void disable() {
    motor.set(0);
  }

  // Getters
  protected CANSparkMax getMotor() { return motor; }
  protected SparkPIDController getController() { return pidController; }
  protected AbsoluteEncoder getEncoder() { return encoder; }
}

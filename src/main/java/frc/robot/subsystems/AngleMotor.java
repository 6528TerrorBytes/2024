// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AngleMotor extends SubsystemBase {
  private final CANSparkMax motor;
  private final SparkPIDController pidController;

  // I wish Typescript unions were a thing in Java
  private AbsoluteEncoder absoluteEncoder;
  private RelativeEncoder relativeEncoder;
  private boolean useRelativeEncoder;

  private double angleGoal = 0;
  private double tolerance = 10;

  private double minAngle;
  private double maxAngle;

  /** Creates a new AngleMotor. */
  public AngleMotor(int motorID, double minAngle, double maxAngle, boolean useRelativeEncoder) {
    this.useRelativeEncoder = useRelativeEncoder;
    motor = new CANSparkMax(motorID, MotorType.kBrushless);
    pidController = motor.getPIDController();

    if (!useRelativeEncoder) {
      absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

      // Setup encoder conversions to use radians and radians per second
      absoluteEncoder.setPositionConversionFactor(Constants.toDegrees);
      absoluteEncoder.setVelocityConversionFactor(Constants.toDegreesPerSec);

      pidController.setFeedbackDevice(absoluteEncoder);
    } else {
      relativeEncoder = motor.getEncoder();

      relativeEncoder.setPositionConversionFactor(Constants.toDegrees);
      relativeEncoder.setVelocityConversionFactor(Constants.toDegreesPerSec);

      pidController.setFeedbackDevice(relativeEncoder);
    }

    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
        
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingMaxInput(360);
    pidController.setPositionPIDWrappingEnabled(true);
  }
  
  public void burnFlash() {
    motor.burnFlash();
  }

  // Need to burn flash after running this function
  public void setSpeed(double speed) {
    // Control speed from -1 to 1 (min/max motor speed)
    pidController.setOutputRange(-speed, speed);
  }

  // Moves to the given angle (in degrees)
  public void setGoal(double angle) {
    angleGoal = angle;

    // Clamp the angle goal between the angle limits
    if (angleGoal < minAngle) {
      System.out.println("Hit minimum angle: " + Double.toString(minAngle));
      return;
    }
    
    if (angleGoal > maxAngle) {
      System.out.println("Hit max angle: " + Double.toString(maxAngle));
      return;
    }

    if (Double.isNaN(angleGoal)) {
      System.out.println("Hit a NaN");
      return;
    }

    // Set the PID controller to move to the angle
    System.out.println("Goal: " + Double.toString(angleGoal));
    pidController.setReference(angleGoal, CANSparkMax.ControlType.kPosition);
  }

  public void setTolerance(double angleOffset) {
    tolerance = angleOffset;
  }

  public double getAngle() {
    if (useRelativeEncoder) {
      return relativeEncoder.getPosition();
    } else {
      return absoluteEncoder.getPosition();
    }
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

  // Check in the execute function of a command!
  public void check() { // DISABLE BEFORE COMPETITION
    double encoderPos = getAngle();
    if (encoderPos < minAngle || encoderPos > maxAngle) {
      disable();
    }
  }

  // Getters
  protected CANSparkMax getMotor() { return motor; }
  protected SparkPIDController getController() { return pidController; }
  protected AbsoluteEncoder getAbsoluteEncoder() { return absoluteEncoder; }
  protected RelativeEncoder getRelativeEncoder() { return relativeEncoder; }
}

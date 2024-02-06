// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import frc.robot.Constants;

public class ShooterTiltSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax tiltMotor = new CANSparkMax(Constants.MotorIDs.shooterTilt, MotorType.kBrushless);
  private final AbsoluteEncoder tiltEncoder = tiltMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final ArmFeedforward tiltFeedforward = new ArmFeedforward(
    Constants.ShooterConstants.tiltFeedForwardS,
    Constants.ShooterConstants.tiltFeedForwardG,
    Constants.ShooterConstants.tiltFeedForwardV
  );

  /** Creates a new ShooterTiltSubsystem. */
  public ShooterTiltSubsystem() {
    super(
      // The ProfiledPIDController used by the subsystem
      new ProfiledPIDController(
        Constants.ShooterConstants.tiltP,
        Constants.ShooterConstants.tiltI,
        Constants.ShooterConstants.tiltD,

        // The motion profile constraints
        new TrapezoidProfile.Constraints(
          Constants.ShooterConstants.tiltMaxVelocity,
          Constants.ShooterConstants.tiltMaxAcceleration
        )
      ),
      0
    );
    
    // Set initial goal of 0
    setGoal(0);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    double feedforward = tiltFeedforward.calculate(setpoint.position, setpoint.velocity);

    tiltMotor.setVoltage(output + feedforward);
  }
  
  // Issue: useOuput is setting voltage, when the output is the actual position of the arm. I think.
  // because getMeasurement is returning a position.

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return tiltEncoder.getPosition();
  }

  public void setTolerance(double positionTolerance, double velocityTolerance) {
    getController().setTolerance(positionTolerance, velocityTolerance);
  }

  // Reached the goal within the set tolerance.
  // Set the goal by calling .setGoal()
  public boolean reachedGoal() {
    return getController().atGoal();
  }
}

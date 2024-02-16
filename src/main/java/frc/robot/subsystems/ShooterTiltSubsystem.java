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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkAbsoluteEncoder.Type;
import frc.robot.Constants;

public class ShooterTiltSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax tiltMotor = new CANSparkMax(Constants.MotorIDs.shooterTilt, MotorType.kBrushless);
  
  // Duty Cycle Encoder for Rev-11-1271 Through Bore Encoder/
  // from the "DIO Pin 0"
  private final DutyCycleEncoder tiltEncoder = new DutyCycleEncoder(0);

  private final ArmFeedforward tiltFeedforward = new ArmFeedforward(
    Constants.ShooterConstants.tiltFeedforwardkSVolts,
    Constants.ShooterConstants.tiltFeedforwardkGVolts,
    Constants.ShooterConstants.tiltFeedforwardkVVoltsSecPerRad
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

    // Set encoder to output radians when calling getDistance (2pi = one rotation)
    // tiltEncoder.setDistancePerRotation(Constants.ShooterConstants.toRadians);
    
    // Set initial goal of the horizontal neutral position
    // setGoal(Constants.ShooterConstants.tiltHorizontalOffset);
    setGoal(0);
    
    System.out.println("Shooter Tilt Subsystem enabled: ");
    System.out.println(isEnabled());
    // If it isn't enabled, then call enable();
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
    return tiltEncoder.getDistance(); // Distance rotated (radians)
      // Constants.ShooterConstants.tiltHorizontalOffset; 
  }

  // Sets the offsets of position and velocity around the goal
  // ie. if position tolerance is 
  public void setTolerance(double positionTolerance, double velocityTolerance) {
    getController().setTolerance(positionTolerance, velocityTolerance);
  }

  // Reached the goal within the set tolerance.
  // (Set the goal by calling .setGoal())
  public boolean atGoal() {
    return getController().atGoal();
  }
}

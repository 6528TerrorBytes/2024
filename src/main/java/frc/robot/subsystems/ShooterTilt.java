package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ShooterTilt extends AngleMotor {
  /** Creates a new ShooterTilt. */
  public ShooterTilt() {
    super(
      Constants.MotorIDs.shooterTilt,
      Constants.ShooterConstants.minAngle,
      Constants.ShooterConstants.maxAngle
    );

    setStartingOffset(Constants.ShooterConstants.angleOffset);

    getController().setP(Constants.ShooterConstants.tiltP, 0);
    getController().setI(Constants.ShooterConstants.tiltI, 0);
    getController().setD(Constants.ShooterConstants.tiltD, 0);
    getController().setFF(0, 0); // Feedforward gains (?)
    setSpeed(Constants.ShooterConstants.speed);

    // getController().setSmartMotionMaxAccel(1, 0);
    // getController().setSmartMotionMaxVelocity(5, 0);
    
    getMotor().setInverted(true);
    getMotor().setIdleMode(IdleMode.kBrake);
    getMotor().setSmartCurrentLimit(30); // Just torque (strength), not speed

    burnFlash(); // Save settings

    setTolerance(Constants.ShooterConstants.tolerance);
  }
  
  @Override
  public double getAngle() {
    double angle = super.getAngle();
    SmartDashboard.putNumber("Shooter encoder angle ", angle);
    return angle;
  }
}

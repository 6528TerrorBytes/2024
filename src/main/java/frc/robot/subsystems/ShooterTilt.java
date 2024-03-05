package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ShooterTilt extends AngleMotor {
  private final DigitalInput m_lowerSwitch = new DigitalInput(Constants.LimitSwitches.lowerShooterTilt);
  private final DigitalInput m_upperSwitch = new DigitalInput(Constants.LimitSwitches.upperShooterTilt);
  
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
    getMotor().setSmartCurrentLimit(15); // Just torque (strength), not speed

    burnFlash(); // Save settings

    setTolerance(Constants.ShooterConstants.tolerance);
  }

  public void outputEncoder() {
    SmartDashboard.putNumber("Shooter encoder angle ", getAngle());
  }

  public void testSwitches() {
    // if (!m_lowerSwitch.get()) { 
    //   System.out.println("lower");
    //   disable(); 
    // }
    // if (!m_upperSwitch.get()) { 
    //   System.out.println("upper");
    //   disable(); 
    // }

    // SmartDashboard.putBoolean("lower", !m_lowerSwitch.get());
    // SmartDashboard.putBoolean("upper", !m_upperSwitch.get());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Conversions
  public static final double toDegrees = 360;
  public static final double toDegreesPerSec = 360 / 60;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIDs {
    public static final int intake = 1;

    public static final int conveyerLeft = 3;
    public static final int conveyerRight = 2;

    public static final int shooterTilt = 4;

    public static final int shooterLeft = 5;
    public static final int shooterRight = 6;

    public static final int stopNote = 7;

    public static final int hangerArmLeft = 8;
    public static final int hangerArmRight = 9;
  }

  public static class LimitSwitches {
    public static final int rightHangerArm = 0;
    public static final int leftHangerArm = 1;
    public static final int detectNote = 2;
    public static final int upperShooterTilt = 3;
    public static final int lowerShooterTilt = 4;
  }

  public static class AutonConstants {
    // +- TX value for which the auton horizontal rotate will stop when it reaches 
    public static final double aprilTagHorizontalEndRange = 2.5;

    // The time allowed to speed up the shooter in seconds before it fires
    public static final double speedUpShooterSeconds = 1;
    public static final double conveyerRunSeconds = 0.5;

    public static final double ampSpeedUpSeconds = 0.4;
    public static final double ampConveyerRunSeconds = 1;


    // Auton driving
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
    public static final double kIXController = 0;
    public static final double kIYController = 0;
    public static final double kIThetaController = 0; // Try changing this for more accurate trajectories

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
    );
  }

  public static class ShooterConstants {
    public static final double angleOffset = 0; // Angle offset to vertical arm
    public static final double minAngle = 0;
    public static final double maxAngle = 85;

    // Things to do here:
    // PID Values for shooter tilt (Mar 1):
    // - Set P to 0.01, put I at like 0.00000001 and test it
    // - Also test Smart Current Limit to 1 and work my way up
    //   to limit the torque (power) of the motor

    // You can see graphs in REV Hardware Client, Run -> View graph,
    // with Duty Cycle Position and Duty Cycle Velocity

    // PID controller
    public static final double tiltP = 0.015;
    public static final double tiltI = 0.0000001;
    public static final double tiltD = 0;
    public static final double speed = 1;
    public static final double tolerance = 2;

    // In meters
    public static final double tiltMaxVelocity = 3;
    public static final double tiltMaxAcceleration = 3;

    // Feedforward parameters
    public static final double tiltFeedforwardkSVolts = 1;
    public static final double tiltFeedforwardkGVolts = 0;
    public static final double tiltFeedforwardkVVoltsSecPerRad = 0;

    // Angle for bringing the arm all the way up
    public static final double angleAtVertical = 2;


    // Aiming constants
    // Angle of the arm from vertical in radians
    public static final double limelightAngle = 60 * (Math.PI / 180);
    // In meters, the distance from the AprilTag to the speaker
    public static final double distTagToSpeaker = 0.62;

    // Initial velocity in meters per second of the shooter
    public static final double initialVel = 8.35;
    public static final double gravity = 9.81;

    // Length of the shooter in meters
    public static final double shooterLength = 0.3937;

    public static final double encoderAngleToHorizontal = 87;


    // Constants for rotating the bot horizontally to face AprilTag
    // Dividers for the cubic and linear parts, respectively
    public static final double cubicDivider = 5000;
    public static final double linearDivider = 25;
    
    // Place at which the two functions would cross
    public static final double crossPoint = Math.sqrt(cubicDivider / linearDivider); 

    // After calculations, multiply the speed by this speed scale
    public static final double speedScale = 0.35;
  }

  public static class StopNoteConstants {
    // Typically has a 3 degree offset from the goal for some reason.
    public static final double p = 0.01;
    public static final double i = 0;
    public static final double d = 0;
    public static final double speed = 0.5;
    public static final double tolerance = 10;

    public static final double minAngle = 0;
    public static final double maxAngle = 90;

    public static final double openAngle = 5;
    public static final double closedAngle = 88;
  }

  public static class HangerArmConstants {
    public static final double p = 1;
    public static final double i = 0;
    public static final double d = 0;
    public static final double speed = 1;
    public static final double tolerance = 10;

    public static final double minAngle = 0;
    public static final double maxAngle = 90;

    public static final double upperEncoderLimit = 80;
  }

  public static class BlinkinConstants {
    public static final int id = 0;

    // Color numbers PDF at the bottom of: https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
    public static final double blue = 0.87;
    public static final double red = 0.61;
    public static final double green = 0.77;

    public static final double strobeRed = -0.11;
    public static final double strobeBlue = -0.09;
  }

  // default copy-pasted stuff here:
  // https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 100; // radians per second
    public static final double kMagnitudeSlewRate = 100; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 100; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.75);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(29.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 12;
    public static final int kFrontRightDrivingCanId = 10;
    public static final int kRearRightDrivingCanId = 13;

    public static final int kFrontLeftTurningCanId = 21;
    public static final int kRearLeftTurningCanId = 22;
    public static final int kFrontRightTurningCanId = 20;
    public static final int kRearRightTurningCanId = 23;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}

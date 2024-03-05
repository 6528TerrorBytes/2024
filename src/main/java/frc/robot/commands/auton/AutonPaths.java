// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AimShooter;
import frc.robot.commands.intake.ConveyerComand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.teleop.StopNoteCommand;
import frc.robot.commands.teleop.TiltShooterCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DetectNote;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTilt;
import frc.robot.subsystems.StopNote;

/* Based heavily off of "FRC 0 to Autonomous" (on Youtube) */
public final class AutonPaths {
  // Defines the max speed and acceleration during the autonomous
  public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    AutonConstants.kMaxSpeedMetersPerSecond,
    AutonConstants.kMaxAccelerationMetersPerSecondSquared
  ).setKinematics(DriveConstants.kDriveKinematics);

  // PID controllers used for following the trajectory (correcting errors)
  public static final PIDController xController = new PIDController(AutonConstants.kPXController, 0, 0);
  public static final PIDController yController = new PIDController(AutonConstants.kPYController, 0, 0);
  // Angle correction PID Controller
  public static final ProfiledPIDController thetaController = new ProfiledPIDController(
    AutonConstants.kPThetaController, 0, 0, AutonConstants.kThetaControllerConstraints
  );

  public static SwerveControllerCommand genSwerveCommand(Trajectory trajectory, DriveSubsystem robotDrive) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Handles the swerve trajectory stuff
    return new SwerveControllerCommand(
      trajectory,
      robotDrive::getPose,
      DriveConstants.kDriveKinematics,
      xController, yController,
      thetaController,
      robotDrive::setModuleStates,
      robotDrive
    );
  }

  public static Trajectory genTrajectory(List<Pose2d> points) {
    // Trajectory path of points that the robot will follow in autonomous.
    // Uses a quintic spline rather than cubic,
    // see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html
    return TrajectoryGenerator.generateTrajectory(
      points, trajectoryConfig
    );
  }

  public static Trajectory genTrajectory(Pose2d starting, List<Translation2d> points, Pose2d ending) {
    // Uses a cubic spline
    return TrajectoryGenerator.generateTrajectory(
      starting, points, ending trajectoryConfig
    );
  }

  public static InstantCommand resetOdometryCommand(DriveSubsystem robotDrive, Trajectory trajectory) {
    // MAY CAUSE ISSUES WITH THE ROBOT'S GYRO. TEST IT WHEN YOU CAN
    return new InstantCommand(() -> robotDrive.resetOdometry(trajectory.getInitialPose())); // Reset the odometry of the bot to 0, 0, 0
  }

  public static Command createDefaultAuton(DriveSubsystem robotDrive) {
    Trajectory trajectory = genTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(0.5, 0.5)
      ),
      new Pose2d(1, 0, Rotation2d.fromDegrees(-90))
    );

    SwerveControllerCommand swerveCommand = genSwerveCommand(trajectory, robotDrive);
    AutonRotate autonRotate = new AutonRotate(robotDrive);
    
    // Runs these three things in order as a single command
    return new SequentialCommandGroup(
      new InstantCommand(() -> robotDrive.resetOdometry(trajectory.getInitialPose())), 
      swerveCommand, // Run the swerve auton with the trajectory
      autonRotate, // Rotates the bot 45 degrees maybe
      new InstantCommand(() -> robotDrive.setX()) // Sets wheels to X positions after
    );
  }

  public static Command createMainAuton(
    DriveSubsystem robotDrive,
    ShooterTilt shooterTilt,
    StopNote stopNote,
    ConveyerSubsystem conveyerSubsystem,
    ShooterSubsystem shooterSubsystem,
    DetectNote detectNote,
    IntakeSubsystem intakeSubsystem
  ) {
    // Generate trajectories
    Trajectory moveOneMeter = genTrajectory(List.of(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(1.5, 0, Rotation2d.fromDegrees(-45))
    ));
    SwerveControllerCommand moveOneMeterCommand = genSwerveCommand(moveOneMeter, robotDrive);
    // FollowTrajectory moveOneMeterCommand = new FollowTrajectory(robotDrive, moveOneMeter, -45);

    Trajectory moveTwoMeters = genTrajectory(List.of(
      new Pose2d(1.5, 0, Rotation2d.fromDegrees(-45)),
      new Pose2d(3.5, 0, Rotation2d.fromDegrees(0))
    ));
    SwerveControllerCommand moveTwoMetersCommand = genSwerveCommand(moveTwoMeters, robotDrive);
    // FollowTrajectory moveOneMeterCommand = new FollowTrajectory(robotDrive, moveTwoMeters, 0);

    Command resetOdometry = resetOdometryCommand(robotDrive, moveOneMeter);

    // Runs these commands in order
    return new SequentialCommandGroup(
      resetOdometry,
      moveOneMeterCommand, // Moves one meter out

      // Output odometry values to see if they're accurate
      new InstantCommand(() -> {
        Pose2d pose = robotDrive.getPose();

        System.out.println("Angle, x, and y after first meter move");
        System.out.println(robotDrive.getRawAngle());
        System.out.println(pose.getX());
        System.out.println(pose.getY());
      })
      
      new ParallelCommandGroup(
        new StopNoteCommand(stopNote, false), // Making sure the stop note goes up
        new SpeedUpShooter(shooterSubsystem, 1, Constants.AutonConstants.speedUpShooterSeconds),
        new AimShooter(shooterTilt, true) // Then aims the shooter up to the speaker
      ),
      
      new FireShooter(conveyerSubsystem, shooterSubsystem, Constants.AutonConstants.conveyerRunSeconds),
      // new TiltShooterCommand(shooterTilt, 3),

      new ParallelDeadlineGroup( // Ends when the conveyer command ends, when a note has been detected
        new ConveyerComand(conveyerSubsystem, detectNote, 1, true),

        new StopNoteCommand(stopNote, true), // Brings note stopper down
        new IntakeCommand(intakeSubsystem, 1), // Run intake
        new SequentialCommandGroup(
          moveTwoMetersCommand, // Move 2 meters back
          new InstantCommand(() -> robotDrive.setX()) // and then stop 
        )
      ),

      new InstantCommand(() -> robotDrive.setX()),
      
      new AutonFaceAprilTag(robotDrive),
      new ParallelCommandGroup(
        new StopNoteCommand(stopNote, false), // Making sure the stop note goes up
        new SpeedUpShooter(shooterSubsystem, 1, Constants.AutonConstants.speedUpShooterSeconds),
        new AimShooter(shooterTilt, true) // Then aims the shooter up to the speaker
      ),

      new FireShooter(conveyerSubsystem, shooterSubsystem, Constants.AutonConstants.conveyerRunSeconds)
    );
  }
}

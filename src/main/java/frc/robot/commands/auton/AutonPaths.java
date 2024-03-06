// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AimShooter;
import frc.robot.commands.intake.ConveyerCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.teleop.StopNoteCommand;
import frc.robot.commands.teleop.TiltShooterCommand;
import frc.robot.commands.auton.groups.AimAndShoot;
import frc.robot.commands.auton.groups.MoveAndIntake;
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
  public static final PIDController xController = new PIDController(
    AutonConstants.kPXController, AutonConstants.kIXController, 0
  );
  public static final PIDController yController = new PIDController(
    AutonConstants.kPYController, AutonConstants.kIYController, 0
  );
  // Angle correction PID Controller
  public static final ProfiledPIDController thetaController = new ProfiledPIDController(
    AutonConstants.kPThetaController, AutonConstants.kIThetaController, 0, 
    AutonConstants.kThetaControllerConstraints
  );

  // Autonomous selector
  public static final String smallCornerAuton = "smallCorner";
  public static final String speakerCenterAuton = "speakerCenter";
  public static final String bigSideAuton = "bigSide";
  public static final SendableChooser<String> chooser = new SendableChooser<>();

  public static void setupAutonChooser() {
    chooser.setDefaultOption("Small Corner Auton", smallCornerAuton);
    chooser.addOption("Speaker Center Auton", speakerCenterAuton);
    chooser.addOption("Big Side Auton", bigSideAuton);
    SmartDashboard.putData("Select auton", chooser);
  }

  public static String getAuton() {
    return chooser.getSelected();
  }

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
      starting, points, ending, trajectoryConfig
    );
  }

  public static InstantCommand resetOdometryCommand(DriveSubsystem robotDrive, Trajectory trajectory) {
    // MAY CAUSE ISSUES WITH THE ROBOT'S GYRO. TEST IT WHEN YOU CAN
    return new InstantCommand(() -> robotDrive.resetOdometry(trajectory.getInitialPose())); // Reset the odometry of the bot to 0, 0, 0
  }

  public static InstantCommand setDriveX(DriveSubsystem robotDrive) {
    return new InstantCommand(() -> robotDrive.setX());
  }

  public static InstantCommand outputPoseCommand(DriveSubsystem robotDrive) {
    return new InstantCommand(() -> {
      Pose2d pose = robotDrive.getPose();

      System.out.println("Angle, x, and y (respectively):");
      System.out.println(robotDrive.getRawAngle());
      System.out.println(pose.getX());
      System.out.println(pose.getY());
    });
  }

  // Robot rotates to your right (when facing it from intake) when direction is -1
  public static Command createMainAuton(
    int direction, int numberRings,
    DriveSubsystem robotDrive,
    ShooterTilt shooterTilt,
    StopNote stopNote,
    ConveyerSubsystem conveyerSubsystem,
    ShooterSubsystem shooterSubsystem,
    DetectNote detectNote,
    IntakeSubsystem intakeSubsystem
  ) {
    // Generate trajectories
    Trajectory firstMove = genTrajectory(List.of(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(1.5, 0, Rotation2d.fromDegrees(40 * direction))
    ));
    SwerveControllerCommand firstMoveCommand = genSwerveCommand(firstMove, robotDrive);
    // FollowTrajectory moveOneMeterCommand = new FollowTrajectory(robotDrive, moveOneMeter, -45);

    robotDrive.setFieldTrajectory(firstMove); // Show on SmartDashboard/Glass

    Trajectory secondMove = genTrajectory(List.of(
      new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(2.7, 0, Rotation2d.fromDegrees(0))
    ));
    SwerveControllerCommand secondMoveCommand = genSwerveCommand(secondMove, robotDrive);
    
    robotDrive.setFieldTrajectory(secondMove);

    // Runs these commands in order
    SequentialCommandGroup firstTwoRings = new SequentialCommandGroup(
      // Moves one meter out
      resetOdometryCommand(robotDrive, firstMove),
      firstMoveCommand,
      outputPoseCommand(robotDrive),
      
      // Aim and shoot!
      new AimAndShoot(stopNote, shooterSubsystem, shooterTilt, conveyerSubsystem),
      // new TiltShooterCommand(shooterTilt, 3),
      
      // Rotate back to zero and then move and pick up a ring
      new AutonRotate(robotDrive, 0),
      new MoveAndIntake(
        secondMoveCommand,
        stopNote, detectNote, conveyerSubsystem, intakeSubsystem
      ),

      outputPoseCommand(robotDrive),

      // Rotate a bit, aim to AprilTag horizontally, then aim vertically and shoot
      new AutonRotate(robotDrive, -35 * direction),
      new AutonFaceAprilTag(robotDrive),
      new AimAndShoot(stopNote, shooterSubsystem, shooterTilt, conveyerSubsystem),
      
      outputPoseCommand(robotDrive)
    );

    if (numberRings <= 2) {
      return new SequentialCommandGroup(firstTwoRings, new TiltShooterCommand(shooterTilt, 3)); // Tilting up
    }
    
    Trajectory thirdMove = genTrajectory(List.of(
      new Pose2d(0, 0, Rotation2d.fromDegrees(-90 * direction)),
      new Pose2d(1.5, 0, Rotation2d.fromDegrees(-90 * direction))
    ));
    SwerveControllerCommand thirdMoveCommand = genSwerveCommand(thirdMove, robotDrive);

    SequentialCommandGroup secondTwoRings = new SequentialCommandGroup(
      resetOdometryCommand(robotDrive, thirdMove),

      new AutonRotate(robotDrive, 90 * direction),
      new MoveAndIntake(
        thirdMoveCommand,
        stopNote, detectNote, conveyerSubsystem, intakeSubsystem
      ),
      
      outputPoseCommand(robotDrive)
      
      // new AutonRotate(robotDrive, 0),
      // new AutonFaceAprilTag(robotDrive),
      // new AimAndShoot(stopNote, shooterSubsystem, shooterTilt, conveyerSubsystem),
      // new TiltShooterCommand(shooterTilt, 3)
    );

    System.out.println("third ring");

    return new SequentialCommandGroup(firstTwoRings, secondTwoRings);
  }

  public static Command createCenteredAuton(
    DriveSubsystem robotDrive,
    ShooterTilt shooterTilt,
    StopNote stopNote,
    ConveyerSubsystem conveyerSubsystem,
    ShooterSubsystem shooterSubsystem,
    DetectNote detectNote,
    IntakeSubsystem intakeSubsystem
  ) {
    Trajectory moveOut = genTrajectory(List.of(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      new Pose2d(1.75, 0, Rotation2d.fromDegrees(0))
    ));
    SwerveControllerCommand moveOutCommand = genSwerveCommand(moveOut, robotDrive);

    return new SequentialCommandGroup(
      resetOdometryCommand(robotDrive, moveOut),

      // Aim and shoot the initial ring (only aiming vertically)
      new AimAndShoot(stopNote, shooterSubsystem, shooterTilt, conveyerSubsystem),

      // Moves out the distance to pick up a ring
      new MoveAndIntake(
        moveOutCommand,
        stopNote, detectNote, conveyerSubsystem, intakeSubsystem
      ),

      setDriveX(robotDrive),      
      outputPoseCommand(robotDrive),

      // Fire shooter and bring it back up after
      new AimAndShoot(stopNote, shooterSubsystem, shooterTilt, conveyerSubsystem),
      new TiltShooterCommand(shooterTilt, 3)
    );
  }
}

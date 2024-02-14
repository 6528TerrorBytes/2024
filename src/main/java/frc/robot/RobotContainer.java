// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ChangeSpeed;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ConveyerSubsystem;

// Autonomous imports
import java.util.List;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterTilt;

import frc.robot.commands.auton.AutonRotate;
import frc.robot.commands.intake.ConveyerComand;
import frc.robot.commands.intake.DecreaseSpeed;
import frc.robot.commands.intake.IncreaseSpeed;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.ReverseConveyerCommand;
import frc.robot.commands.intake.ReverseIntakeCommand;
import frc.robot.commands.intake.SlowIntakeCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final Joystick leftJoystick = new Joystick(1);
  public final Joystick rightJoystick = new Joystick(0);

  // intake, shooter, and other stuff (and aedan)
  public final Joystick otherJoystick = new Joystick(2);

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final ChangeSpeed m_changeSpeed = new ChangeSpeed();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  // private final ShooterTilt m_shooterTilt = new ShooterTilt();

  private final ConveyerSubsystem m_ConveyerSubsystem = new ConveyerSubsystem();

  // Configure information based on the driver station Team Station
  public static final DriverStation.Alliance teamColor = DriverStation.getAlliance().get();
  public static int teamLocation = DriverStation.getLocation().getAsInt();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    System.out.println(teamColor == DriverStation.Alliance.Blue);
    System.out.println(teamLocation);

    // Default command
    // m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
    //     new RunCommand(
    //         () -> m_robotDrive.drive(
    //             -MathUtil.applyDeadband(rightJoystick.getY(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(rightJoystick.getX(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(leftJoystick.getZ(), OIConstants.kDriveDeadband),
    //             true, true),
    //         m_robotDrive)
    //         // Call of duty (:<
    // );

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Joystick buttons
    // new JoystickButton(rightJoystick, 2).whileTrue(new IncreaseSpeed(m_changeSpeed));

    new JoystickButton(leftJoystick, 1).whileTrue(new ConveyerComand(m_ConveyerSubsystem));
    new JoystickButton(leftJoystick, 2).whileTrue(new ReverseConveyerCommand(m_ConveyerSubsystem));
    
    new JoystickButton(rightJoystick, 1).whileTrue(new ParallelCommandGroup(
        new IntakeCommand(m_intakeSubsystem),
        new ConveyerComand(m_ConveyerSubsystem)
    ));

    new JoystickButton(rightJoystick, 2).whileTrue(new ReverseIntakeCommand(m_intakeSubsystem));
    new JoystickButton(otherJoystick, 1).whileTrue(new SlowIntakeCommand(m_intakeSubsystem));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutonRotate(m_robotDrive);
    /* 
    // Based heavily off of "FRC 0 to Autonomous"
    System.out.println("Making Autonomous...");

    // Configuration for the trajectory (max speed and acceleration)
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(DriveConstants.kDriveKinematics);

    // Trajectory path of points that the robot will follow in autonomous
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(0.5, 0.5)
      ),
      new Pose2d(1, 0, Rotation2d.fromDegrees(-90)),
      trajectoryConfig
    );

    // PID controllers used for following the trajectory (correcting errors)
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    // Angle correction PID Controller
    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Handles the swerve trajectory stuff
    SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
      trajectory,
      m_robotDrive::getPose,
      DriveConstants.kDriveKinematics,
      xController, yController,
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive
    );

    AutonRotate autonRotate = new AutonRotate(m_robotDrive);

    System.out.println("Finished");
    
    // Runs these three things in order as a single command
    return new SequentialCommandGroup(
      // new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory.getInitialPose())), // Reset the odometry of the bot to 0, 0, 0
      // swerveCommand, // Run the swerve auton with the trajectory
      // autonRotate // Rotates the bot 45 degrees maybe
      // new InstantCommand(() -> m_robotDrive.setX()) // Sets wheels to X positions
    );
    */
  }
}

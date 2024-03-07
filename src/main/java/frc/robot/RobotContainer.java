// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangerArm;
import frc.robot.subsystems.StopNote;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DetectNote;

// Autonomous imports
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AimShooter;
import frc.robot.commands.NoteBlinkinColor;
import frc.robot.commands.auton.AutonFaceAprilTag;
import frc.robot.commands.auton.AutonPaths;
import frc.robot.commands.auton.FireShooter;
import frc.robot.commands.auton.SpeedUpShooter;
import frc.robot.commands.intake.ConveyerCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.teleop.ExtendHangerArms;
import frc.robot.commands.teleop.StopNoteCommand;
import frc.robot.commands.teleop.TeleopFaceAprilTag;
import frc.robot.subsystems.ShooterTilt;
import frc.robot.commands.teleop.TiltShooterCommand;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.teleop.ShooterCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final Joystick leftJoystick = new Joystick(0);
  public final Joystick rightJoystick = new Joystick(1);

  // intake, shooter, and other stuff (and aedan)
  public final Joystick otherJoystick = new Joystick(2);

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final boolean teleopDrive = true;

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  
  private final ConveyerSubsystem m_conveyerSubsystem = new ConveyerSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  public final ShooterTilt m_shooterTilt = new ShooterTilt();
  private final StopNote m_stopNote = new StopNote();

  public final HangerArm m_hangerArm = new HangerArm();

  private final DetectNote m_detectNote = new DetectNote();

  private final Blinkin m_blinkin = new Blinkin();

  private final NoteBlinkinColor m_blinkinCommand;
  
  // Configure information based on the driver station Team Station
  public static final DriverStation.Alliance teamColor = DriverStation.getAlliance().get();
  public static int teamLocation = DriverStation.getLocation().getAsInt();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    System.out.println(teamColor == DriverStation.Alliance.Blue);
    System.out.println(teamLocation);

    // Drive command
    if (teleopDrive) {
      m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
          () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(rightJoystick.getY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(rightJoystick.getX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(leftJoystick.getZ(), OIConstants.kDriveDeadband),
            true, true),
          m_robotDrive
        )
      ); // Call of duty (:<
    }

    // Updates the blinkin color depending on whether there's a note in it
    m_blinkinCommand = new NoteBlinkinColor(m_blinkin, m_detectNote);
    m_blinkin.setDefaultCommand(m_blinkinCommand);

    // Configure the trigger bindings
    finalControllerBindings();
  }

  private void finalControllerBindings() {
    // ---------- LEFT  JOYSTICK ----------

    // Runs intake, conveyer, and stop note down (back button)
    new JoystickButton(rightJoystick, 1).whileTrue(new ParallelCommandGroup(
      new StopNoteCommand(m_stopNote, true),
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, 0.75, true),
      new IntakeCommand(m_intakeSubsystem, 1)
    ));
    
    // Reverse intake (front bottom center button)
    new JoystickButton(rightJoystick, 2).whileTrue(new ParallelCommandGroup(
      new StopNoteCommand(m_stopNote, false),
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, -0.75, false),
      new IntakeCommand(m_intakeSubsystem, -0.75)
    ));

    // Climbers (up: front left button, retract: front right button)
    new JoystickButton(rightJoystick, 3).whileTrue(new ExtendHangerArms(m_hangerArm, false));
    new JoystickButton(rightJoystick, 4).whileTrue(new ExtendHangerArms(m_hangerArm, true));

    // ---------- RIGHT JOYSTICK ----------

    // Teleop face AprilTag (back button), and disable (front bottom center)
    new JoystickButton(leftJoystick, 1).whileTrue(new TeleopFaceAprilTag());
    // new JoystickButton(leftJoystick, 2).onTrue(new InstantCommand(() -> TeleopFaceAprilTag.disable = true));

    // Make wheels into X (bottom of controller, left side, top right button)
    new JoystickButton(leftJoystick, 13).onTrue(new InstantCommand(() -> m_robotDrive.setX()));

    // Stop note up (bottom of controller, left side, bottom right button)
    new JoystickButton(leftJoystick, 14).onTrue(new StopNoteCommand(m_stopNote, false));
    new JoystickButton(leftJoystick, 15).onTrue(new StopNoteCommand(m_stopNote, true));

    // ---------- OTHER JOYSTICK ----------

    // Other driver auto shooter tilt aim (thumb button)
    new JoystickButton(otherJoystick, 2).whileTrue(new AimShooter(m_shooterTilt, false));

    // Hold back button to shoot
    new JoystickButton(otherJoystick, 1).whileTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new StopNoteCommand(m_stopNote, false),
        new SpeedUpShooter(m_shooterSubsystem, 1, Constants.AutonConstants.speedUpShooterSeconds)
      ),

      new FireShooter(m_conveyerSubsystem, m_shooterSubsystem, Constants.AutonConstants.conveyerRunSeconds),
      new TiltShooterCommand(m_shooterTilt, Constants.ShooterConstants.angleAtVertical)
    ));

    // Slow shoot for amp (front top left button)
    new JoystickButton(otherJoystick, 5).whileTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new StopNoteCommand(m_stopNote, false),
        new SpeedUpShooter(m_shooterSubsystem, 0.5, Constants.AutonConstants.ampSpeedUpSeconds)
      ),

      new FireShooter(m_conveyerSubsystem, m_shooterSubsystem, Constants.AutonConstants.ampConveyerRunSeconds),
      new TiltShooterCommand(m_shooterTilt, Constants.ShooterConstants.angleAtVertical)
    ));

    // Bring the shooter up to vertical (front bottom right button)
    new JoystickButton(otherJoystick, 4).whileTrue(new TiltShooterCommand(m_shooterTilt, Constants.ShooterConstants.angleAtVertical));

    // Manual conveyer forward (controller bottom top right)
    new JoystickButton(otherJoystick, 8).whileTrue(new ParallelCommandGroup(
      new StopNoteCommand(m_stopNote, false),
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, 1, false)
    ));

    // Manual conveyer backward (controller bottom top left)
    new JoystickButton(otherJoystick, 7).whileTrue(new ParallelCommandGroup(
      new StopNoteCommand(m_stopNote, false),
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, -1, false)
    ));

    // Manual aim to speaker (front top right button)
    new JoystickButton(otherJoystick, 6).whileTrue(new TiltShooterCommand(m_shooterTilt, 25));
    // Manual aim to amp (front bottom left button)
    new JoystickButton(otherJoystick, 3).whileTrue(new TiltShooterCommand(m_shooterTilt, 17));
  }

  private void oldControllerBindings() {
    new JoystickButton(leftJoystick, 1).whileTrue(new ParallelCommandGroup(
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, 1, false),
      new StopNoteCommand(m_stopNote, false)
    ));
    
    new JoystickButton(rightJoystick, 1).whileTrue(new ParallelCommandGroup(
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, 1, true),
      new IntakeCommand(m_intakeSubsystem, 1),
      new StopNoteCommand(m_stopNote, true)
    ));

    new JoystickButton(leftJoystick, 2).whileTrue(
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, -1, false)
    );

    new JoystickButton(rightJoystick, 2).whileTrue(new IntakeCommand(m_intakeSubsystem, -1));
    // new JoystickButton(otherJoystick, 1).whileTrue(new SlowIntakeCommand(m_intakeSubsystem));

    // new JoystickButton(otherJoystick, 1).onTrue(new TiltShooterCommand(m_shooterTilt, 20));
    new JoystickButton(otherJoystick, 8).onTrue(new TiltShooterCommand(m_shooterTilt, 45));
    new JoystickButton(otherJoystick, 9).onTrue(new TiltShooterCommand(m_shooterTilt, 90));
    new JoystickButton(otherJoystick, 10).onTrue(new TiltShooterCommand(m_shooterTilt, 70));
    new JoystickButton(otherJoystick, 11).onTrue(new TiltShooterCommand(m_shooterTilt, 80));
    new JoystickButton(otherJoystick, 12).onTrue(new TiltShooterCommand(m_shooterTilt, 90));
    
    new JoystickButton(rightJoystick, 14).onTrue(new TiltShooterCommand(m_shooterTilt, Constants.ShooterConstants.angleAtVertical));

    new JoystickButton(leftJoystick, 4).whileTrue(new ShooterCommand(m_shooterSubsystem, 1));
    new JoystickButton(leftJoystick, 3).whileTrue(new ShooterCommand(m_shooterSubsystem, 0.2));
    new JoystickButton(leftJoystick, 13).whileTrue(new ShooterCommand(m_shooterSubsystem, -0.5));
    
    new JoystickButton(rightJoystick, 3).onTrue(new StopNoteCommand(m_stopNote, true));
    new JoystickButton(rightJoystick, 4).onTrue(new StopNoteCommand(m_stopNote, false));

    new JoystickButton(rightJoystick, 10).whileTrue(new ExtendHangerArms(m_hangerArm, true));
    new JoystickButton(rightJoystick, 9).whileTrue(new ExtendHangerArms(m_hangerArm, false));
    new JoystickButton(rightJoystick, 6).onTrue(new TeleopFaceAprilTag());
    new JoystickButton(rightJoystick, 7).onTrue(new AimShooter(m_shooterTilt, false));
  }

  public Command getAutonomousCommand() {
    String autonChosen = AutonPaths.getAuton();
    System.out.println("Making Autonomous: " + autonChosen);

    // Running the autonomous selected in the SmartDashboard
    switch (autonChosen) {
      case AutonPaths.smallCornerAuton: // For starting the robot in the small corner of the field
        return AutonPaths.createMainAuton(
          -1  * (Utility.teamColorIsRed() ? 1 : -1), // Changes direction based on whether the team is read or not
          false, m_robotDrive, m_shooterTilt, m_stopNote, m_conveyerSubsystem, m_shooterSubsystem, m_detectNote, m_intakeSubsystem
        );
      
      case AutonPaths.speakerCenterAuton: // For starting the robot centered against the front of the subwoofer/speaker
        return AutonPaths.createCenteredAuton(
          m_robotDrive, m_shooterTilt, m_stopNote, m_conveyerSubsystem, m_shooterSubsystem, m_detectNote, m_intakeSubsystem
        );
      
      case AutonPaths.bigSideAuton: // For starting the robot on the side of the speaker with a lot of room
        return AutonPaths.createMainAuton(
          1 * (Utility.teamColorIsRed() ? 1 : -1),
          true, m_robotDrive, m_shooterTilt, m_stopNote, m_conveyerSubsystem, m_shooterSubsystem, m_detectNote, m_intakeSubsystem
        );
    }

    System.out.println("INCORRECT SMARTDASHBOARD AUTON SELECTION (somehow)");
    return null; // Something bad happened
  }

  public Command getTestCommand() {
    return new AutonFaceAprilTag(m_robotDrive);
  }

  public void setTeleop(boolean isTeleop) {
    System.out.println("IS TELEOP:");
    System.out.println(isTeleop);
    m_blinkinCommand.setTeleop(isTeleop);
  }
}

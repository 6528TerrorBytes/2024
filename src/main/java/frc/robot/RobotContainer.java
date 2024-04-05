// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HangerArm;
import frc.robot.subsystems.StopNote;
import frc.utils.JoystickAnalogButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AmpFlap;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DetectNote;

// Autonomous imports
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AimShooter;
import frc.robot.commands.NoteBlinkinColor;
import frc.robot.commands.auton.AutonFaceAprilTag;
import frc.robot.commands.auton.AutonPaths;
import frc.robot.commands.auton.FireShooter;
import frc.robot.commands.auton.SpeedUpShooter;
import frc.robot.commands.auton.pathPlanner.EndAutonIntake;
import frc.robot.commands.auton.pathPlanner.StartAutonIntake;
import frc.robot.commands.intake.ConveyerCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.teleop.ExtendHangerArms;
import frc.robot.commands.teleop.OverrideShooterDisable;
import frc.robot.commands.teleop.StopNoteCommand;
import frc.robot.commands.teleop.TeleopFaceAprilTag;
import frc.robot.commands.teleop.AmpFlapCommand;
import frc.robot.commands.teleop.DriveSpeedUp;
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
  private final boolean useXboxControllerForOther = true;

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final boolean teleopDrive = true;

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  
  private final ConveyerSubsystem m_conveyerSubsystem = new ConveyerSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  public final ShooterTilt m_shooterTilt = new ShooterTilt();
  private final StopNote m_stopNote = new StopNote();
  private final AmpFlap m_ampFlap = new AmpFlap();

  public final HangerArm m_hangerArm = new HangerArm();

  private final DetectNote m_detectNote = new DetectNote();

  private final Blinkin m_blinkin = new Blinkin();

  private final NoteBlinkinColor m_blinkinCommand;

  // Configure information based on the driver station Team Station
  public static final DriverStation.Alliance teamColor = DriverStation.getAlliance().get();
  public static int teamLocation = DriverStation.getLocation().getAsInt();
  
  private SendableChooser<String> m_pathPlannnerChooser = new SendableChooser<String>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    registerAutonCommands();

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
            true, true, true),
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
    // ---------- RIGHT JOYSTICK ----------

    // Runs intake, conveyer, and stop note down (trigger/back button)
    new JoystickButton(rightJoystick, 1).whileTrue(new ParallelCommandGroup(
      new StopNoteCommand(m_stopNote, true),
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, 1, true),
      new IntakeCommand(m_intakeSubsystem, m_detectNote, 1)
    ));
    
    // Reverse intake (front bottom center button)
    new JoystickButton(rightJoystick, 2).whileTrue(new ParallelCommandGroup(
      new StopNoteCommand(m_stopNote, false),
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, -1, false),
      new IntakeCommand(m_intakeSubsystem, m_detectNote, -1)
    ));

    // Climbers (up: front left button, retract: front right button)
    new JoystickButton(rightJoystick, 3).whileTrue(new ExtendHangerArms(m_hangerArm, false));
    new JoystickButton(rightJoystick, 4).whileTrue(new ExtendHangerArms(m_hangerArm, true));

    // Stop note up (bottom of controller, left side, bottom right button)
    new JoystickButton(rightJoystick, 14).onTrue(new AmpFlapCommand(m_ampFlap, false));
    new JoystickButton(rightJoystick, 15).onTrue(new AmpFlapCommand(m_ampFlap, true));

    // ---------- LEFT JOYSTICK ----------

    // Teleop face AprilTag (trigger/back button)
    new JoystickButton(leftJoystick, 1).whileTrue(new TeleopFaceAprilTag());
    // Speeds up driving (front bottom center)
    new JoystickButton(leftJoystick, 2).whileTrue(new DriveSpeedUp());

    new JoystickButton(leftJoystick, 11).onTrue(new InstantCommand(() -> m_robotDrive.resetGyro()));

    // Make wheels into X (bottom of controller, left side, top right button)
    new JoystickButton(leftJoystick, 13).onTrue(new InstantCommand(() -> m_robotDrive.setX()));

    // Stop note up (bottom of controller, left side, bottom right button)
    new JoystickButton(leftJoystick, 14).onTrue(new StopNoteCommand(m_stopNote, false));
    new JoystickButton(leftJoystick, 15).onTrue(new StopNoteCommand(m_stopNote, true));

    // ---------- OTHER JOYSTICK ----------
    if (!useXboxControllerForOther) {
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


    } else {
      // XBOX CONTROLLER
      // Back left trigger, aim and speed up shooter
      new JoystickAnalogButton(otherJoystick, 2, 0.5).whileTrue(new ParallelCommandGroup(
        new AimShooter(m_shooterTilt, false),
        new SpeedUpShooter(m_shooterSubsystem, 1, false),
        new StopNoteCommand(m_stopNote, false)
      ));

      // Manual aim to speaker (Y)
      new JoystickButton(otherJoystick, 4).whileTrue(new ParallelCommandGroup(
        new TiltShooterCommand(m_shooterTilt, 22),  
        new SpeedUpShooter(m_shooterSubsystem, 1, false),
        new StopNoteCommand(m_stopNote, false)
      ));
      
      // Backup spool shooter (X)
      // new JoystickButton(otherJoystick, 3).whileTrue(new ParallelCommandGroup(
      //   new SpeedUpShooter(m_shooterSubsystem, 1, false),
      //   new StopNoteCommand(m_stopNote, false)
      // ));
  
      // Back right trigger, shoot
      new JoystickAnalogButton(otherJoystick, 3, 0.5).onTrue(new ParallelCommandGroup(
        new StopNoteCommand(m_stopNote, false),
        new SequentialCommandGroup(
          new FireShooter(m_conveyerSubsystem, m_shooterSubsystem, Constants.AutonConstants.conveyerRunSeconds),
          new TiltShooterCommand(m_shooterTilt, Constants.ShooterConstants.angleAtVertical)
        )
      ));

      // Manual aim to amp, back left bumper
      new JoystickButton(otherJoystick, 5).onTrue(new ParallelCommandGroup(
        new AmpFlapCommand(m_ampFlap, false),
        new TiltShooterCommand(m_shooterTilt, 20)
      ));
  
      // Slow shoot for amp, back right bumper
      new JoystickButton(otherJoystick, 6).whileTrue(new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new SpeedUpShooter(m_shooterSubsystem, 0.4, Constants.AutonConstants.ampSpeedUpSeconds),
          new StopNoteCommand(m_stopNote, false)
        ),
  
        new FireShooter(m_conveyerSubsystem, m_shooterSubsystem, Constants.AutonConstants.ampConveyerRunSeconds),

        new ParallelCommandGroup(
          new AmpFlapCommand(m_ampFlap, true),
          new TiltShooterCommand(m_shooterTilt, Constants.ShooterConstants.angleAtVertical)
        )
      ));
  
      // Bring the shooter up to vertical (A)
      new JoystickButton(otherJoystick, 1).onTrue(new ParallelDeadlineGroup(
        new TiltShooterCommand(m_shooterTilt, Constants.ShooterConstants.angleAtVertical),
        new AmpFlapCommand(m_ampFlap, true),
        new OverrideShooterDisable()
      ));

      // Shoot across field button (B)
      new JoystickButton(otherJoystick, 2).whileTrue(new TiltShooterCommand(m_shooterTilt, 35));
      // Shoot across field other angle (X)
      new JoystickButton(otherJoystick, 3).whileTrue(new TiltShooterCommand(m_shooterTilt, 30));

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
  
    }
  }

  private void oldControllerBindings() {
    new JoystickButton(leftJoystick, 1).whileTrue(new ParallelCommandGroup(
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, 1, false),
      new StopNoteCommand(m_stopNote, false)
    ));
    
    new JoystickButton(rightJoystick, 1).whileTrue(new ParallelCommandGroup(
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, 1, true),
      new IntakeCommand(m_intakeSubsystem, m_detectNote, 1),
      new StopNoteCommand(m_stopNote, true)
    ));

    new JoystickButton(leftJoystick, 2).whileTrue(
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, -1, false)
    );

    new JoystickButton(rightJoystick, 2).whileTrue(new IntakeCommand(m_intakeSubsystem, m_detectNote, -1));
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

  public Command getNewAuton() {
    String option = m_pathPlannnerChooser.getSelected();

    if (option == null) {
      return null;
    }

    return new PathPlannerAuto(option);
  }

  public void setupNewAuton() {
    m_pathPlannnerChooser.addOption("No Auton", null);
    m_pathPlannnerChooser.addOption("Only Shoot", "Only Shoot");
    m_pathPlannnerChooser.addOption("Center 2 Note", "Center 2 Note");
    m_pathPlannnerChooser.addOption("Center 2 Note to Source-side", "Center 2 Note Source-side");
    m_pathPlannnerChooser.addOption("Center 3 Note To Center", "Center 3 Note To Center");
    m_pathPlannnerChooser.addOption("Center 3 Note To Amp-side", "Center 3 Note Amp-side");
    m_pathPlannnerChooser.addOption("Center 3 Note To Source-side", "Center 3 Note Source-side");
    m_pathPlannnerChooser.setDefaultOption("Center 4 Note", "Center 4 Note");
    m_pathPlannnerChooser.setDefaultOption("Center 4 Note To Center", "Center 4 Note To Center");
    m_pathPlannnerChooser.addOption("Amp-side 1 Note Wait", "Amp-side 1 Note Wait");
    m_pathPlannnerChooser.addOption("Amp-side 2 Note", "Amp-side 2 Note");
    m_pathPlannnerChooser.addOption("Amp-side 3 Note", "Amp-side 3 Note");
    m_pathPlannnerChooser.addOption("Source-side Edge 2 Note", "Source-side Edge 2 Note");
    m_pathPlannnerChooser.addOption("Source-side Ring Swiper", "Source-side Ring Swiper");
    m_pathPlannnerChooser.addOption("Source-side 3 Note", "Source-side 3 Note");
    m_pathPlannnerChooser.addOption("Source-side Shoot & Out", "Source-side Shoot & Out");
    
    // m_pathPlannnerChooser.addOption("Top to bottom auton", "TopToBottom");
    // m_pathPlannnerChooser.addOption("Ring collector :)", "RingCollector");
    // m_pathPlannnerChooser.addOption("Bottom notes shoot", "BottomNotesShoot");

    SmartDashboard.putData("Select PathPlaner auton", m_pathPlannnerChooser);
  }

  private void registerAutonCommands() {
    // Register Named Comands for PathPlanner auton - https://pathplanner.dev/pplib-named-commands.html
    NamedCommands.registerCommand("stopNoteDown", new StopNoteCommand(m_stopNote, true));
    NamedCommands.registerCommand("aimShooterForever", new AimShooter(m_shooterTilt, false));

    NamedCommands.registerCommand("intakeUntilNote", new ParallelDeadlineGroup(
      new ConveyerCommand(m_conveyerSubsystem, m_detectNote, 1, true),
      new IntakeCommand(m_intakeSubsystem, m_detectNote, 1),
      new StopNoteCommand(m_stopNote, true)
    ));
    
    // Commands for starting and ending intake
    NamedCommands.registerCommand("startIntake", new ParallelCommandGroup(
      new StartAutonIntake(m_conveyerSubsystem, m_intakeSubsystem),
      new StopNoteCommand(m_stopNote, true)
    ));
    NamedCommands.registerCommand("endIntake", new EndAutonIntake(m_conveyerSubsystem, m_intakeSubsystem, m_detectNote));
    
    // Fires the shooter!
    NamedCommands.registerCommand("fireShooter", new ParallelDeadlineGroup(
      new SequentialCommandGroup(
        new SpeedUpShooter(m_shooterSubsystem, 1, Constants.AutonConstants.speedUpShooterSeconds),
        new FireShooter(m_conveyerSubsystem, m_shooterSubsystem, Constants.AutonConstants.conveyerRunSeconds)
      ),
      new SequentialCommandGroup(
        new WaitCommand(Constants.AutonConstants.speedUpShooterSeconds / 2),
        new StopNoteCommand(m_stopNote, false)
      )
    ));

    NamedCommands.registerCommand("ampShoot", new ParallelDeadlineGroup(
      new SequentialCommandGroup(
        new SpeedUpShooter(m_shooterSubsystem, 0.5, Constants.AutonConstants.ampSpeedUpSeconds),
        new FireShooter(m_conveyerSubsystem, m_shooterSubsystem, Constants.AutonConstants.ampConveyerRunSeconds)
      ),
      new SequentialCommandGroup(
        new WaitCommand(Constants.AutonConstants.ampSpeedUpSeconds / 2),
        new StopNoteCommand(m_stopNote, false)
      )
    ));

    // Potentially combine the SpeedUpShooter process and the EndAutonIntake process to speed this up by 0.5 seconds for each ring

    NamedCommands.registerCommand("zeroShooter", new TiltShooterCommand(m_shooterTilt, Constants.ShooterConstants.angleAtVertical));
    NamedCommands.registerCommand("aimTo45", new TiltShooterCommand(m_shooterTilt, 45));
    NamedCommands.registerCommand("aimTo60", new TiltShooterCommand(m_shooterTilt, 60));

    NamedCommands.registerCommand("resetGyro", new InstantCommand(() -> m_robotDrive.resetGyro()));
  }

  public Command getOldAuton() {
    String autonChosen = AutonPaths.getAuton();
    System.out.println("Making Autonomous: " + autonChosen);

    // Running the autonomous selected in the SmartDashboard
    switch (autonChosen) {
      case AutonPaths.noAuton:
        return null;

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

  public void setupOldAuton() {
    AutonPaths.setupAutonChooser();
    AutonPaths.setupRingNumberChooser();
  }

  public Command getTestCommand() {
    return new AutonFaceAprilTag(m_robotDrive);
  }

  public void setTeleop(boolean isTeleop) {
    m_blinkinCommand.setTeleop(isTeleop);
  }

  public void dashboardStuff() {
    m_detectNote.addToDashboard();
    m_shooterTilt.outputEncoder();
    m_ampFlap.outputEncoder();
  }
}

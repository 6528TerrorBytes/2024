// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.DecreaseSpeed;
import frc.robot.commands.IncreaseSpeed;
import frc.robot.subsystems.ChangeSpeed;

import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;

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


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    // JoystickButton front = new JoystickButton(joystick, 0);

    // front.whileTrue(new ExampleCommand(subsystem));
    
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(rightJoystick.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(rightJoystick.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(leftJoystick.getZ(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive)
            // Call of duty (:<
    );

    // Joystick buttons

    new JoystickButton(leftJoystick, 1).whileTrue(new DecreaseSpeed(m_changeSpeed));
    new JoystickButton(rightJoystick, 2).whileTrue(new IncreaseSpeed(m_changeSpeed));

    new JoystickButton(otherJoystick, 1).whileTrue(new IntakeCommand(m_intakeSubsystem));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(subsystem);
  // }
}

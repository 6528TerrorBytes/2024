// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_testCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.setupNewAuton();

    // Put running commands in the SmartDashboard
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // SmartDashboard updating stuff:
    // Utility.updateSmartDashboard();
    // m_robotContainer.m_hangerArm.outputLimitSwitches();

    m_robotContainer.dashboardStuff();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("Entering disabled mode...");
    Utility.turnOffLimelightLED();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Utility.turnOnLimelightLED();
    stopAll();

    m_autonomousCommand = m_robotContainer.getNewAuton();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.setTeleop(false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Utility.turnOnLimelightLED();

    stopAutonomousCommand();
    stopTestCommand();
    
    m_robotContainer.setTeleop(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    System.out.println("Entering test mode...");
    stopAll();

    m_testCommand = m_robotContainer.getTestCommand();
    m_testCommand.schedule();

    m_robotContainer.setTeleop(false);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    m_robotContainer.setTeleop(false);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  // Stop functions
  public void stopAutonomousCommand() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public void stopTestCommand() {
    if (m_testCommand != null) {
      m_testCommand.cancel();
    }
  }

  public void stopAll() {
    CommandScheduler.getInstance().cancelAll();
  }
}

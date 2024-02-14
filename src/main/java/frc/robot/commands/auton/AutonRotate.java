// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutonRotate extends Command {
  private final DriveSubsystem m_driveSubsystem;

  private int angleGoal;

  /** Creates a new AutonRotate. */
  public AutonRotate(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleGoal = (int)(m_driveSubsystem.getRawAngle() + 45); // Moving 45 degrees
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_driveSubsystem.drive(0, 0, 0.5, true, true);

    boolean canSee = LimelightHelpers.getTV("limelight");
    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");
    double ta = LimelightHelpers.getTA("limelight");

    double id = LimelightHelpers.getFiducialID("limelight");

    Pose3d botpose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    
    SmartDashboard.putBoolean("Can see limelight: ", canSee);
    SmartDashboard.putNumber("limelight id ", id);

    SmartDashboard.putNumber("limelight tx ", tx);
    SmartDashboard.putNumber("limelight ty ", ty);
    SmartDashboard.putNumber("limelight ta ", ta);

    SmartDashboard.putNumber("limelight X ", botpose.getX());
    SmartDashboard.putNumber("limelight Y ", botpose.getY());
    SmartDashboard.putNumber("limelight Z ", botpose.getZ());
    SmartDashboard.putNumber("limelight angle ", botpose.getRotation().getAngle() * (180 / Math.PI));
  }

  // Called once the command ends or is interrupted.
  //call of duty
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return (int)(m_driveSubsystem.getRawAngle()) >= angleGoal;
  }
}

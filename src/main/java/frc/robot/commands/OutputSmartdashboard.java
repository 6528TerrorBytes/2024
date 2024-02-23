// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
// import frc.robot.subsystems.ShooterTilt;

public class OutputSmartdashboard extends Command {
  // Updates the values displayed in the SmartDashboard
  // private final ShooterTilt m_shooterTilt;
  
  /** Creates a new TiltShooterAlternate. */
  public OutputSmartdashboard() {
    // Use addRequirements() here to declare subsystem dependencies.
    // m_shooterTilt = shooterTilt;
    // addRequirements(m_shooterTilt);
  }

  @Override
  public void execute() {
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
}

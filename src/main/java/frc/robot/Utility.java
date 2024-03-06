// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public final class Utility {
  /**
   * Tests if the AprilTag in view is the correct speaker shooter ID
   * @return whether or not the AprilTag is the correct shooter ID
   */
  public static boolean testShooterID() {
    double id = LimelightHelpers.getFiducialID("limelight");

    boolean isRed = teamColorIsRed();
    SmartDashboard.putBoolean("is team color red? ", isRed);

    return (  
      (isRed && id == 4) ||
      (!isRed && id == 7)
    );
  }

  public static boolean teamColorIsRed() {
    return getAlliance() == DriverStation.Alliance.Red; 
  }

  public static boolean aprilTagInView() {
    return LimelightHelpers.getTV("limelight");
  }

  public static Pose3d aprilTagPos() {
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
  }

  public static DriverStation.Alliance getAlliance() {
    return DriverStation.getAlliance().get();
  }

  public static int getTeamLocation() {
    return DriverStation.getLocation().getAsInt();
  }

  public static double clampNum(double num, double min, double max) {
    if (num > max)      { return max; }
    else if (num < min) { return min; }
    else                { return num; }
  }

  public static double getTime() {
    return Timer.getFPGATimestamp();
  }

  public static void updateSmartDashboard() {
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
    SmartDashboard.putNumber("limelight angle ", botpose.getRotation().getZ() * (180 / Math.PI));
  }

  public static double getTX() {
    return LimelightHelpers.getTX("limelight");
  }

  /**
   * Calculates the speed rotation for the DriveSubsystem needed
   * to face the currently visible AprilTag.
   * @return drive speed rotation to face AprilTag
   */
  public static double calcSpeedFaceTag(double tx) {
    double rotationSpeed;
    rotationSpeed = -tx / Constants.ShooterConstants.linearDivider;

    // if (Math.abs(tx) <= Constants.ShooterConstants.crossPoint) {
    //   // Between +- where the functions cross, use linear
    //   rotationSpeed = -tx / Constants.ShooterConstants.linearDivider;
    // } else { 
    //   // Use cubic since it's outside the inner range
    //   rotationSpeed = -Math.pow(tx, 3) / Constants.ShooterConstants.cubicDivider;
    // }

    rotationSpeed *= Constants.ShooterConstants.speedScale;
    return clampNum(rotationSpeed, -1, 1);
  }
}

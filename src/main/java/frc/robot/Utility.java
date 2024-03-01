// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public final class Utility {
  /**
   * Tests if the AprilTag in view is the correct speaker shooter ID
   * @return whether or not the AprilTag is the correct shooter ID
   */
  public static boolean testShooterID() {
    double id = LimelightHelpers.getFiducialID("limelight");

    SmartDashboard.putBoolean("red ", RobotContainer.teamColor == DriverStation.Alliance.Red);

    DriverStation.Alliance alliance = getAlliance();

    return (
      (alliance == DriverStation.Alliance.Blue && id == 7) ||
      (alliance == DriverStation.Alliance.Red && id == 4)
    );
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

  /**
   * Calculates the speed rotation for the DriveSubsystem needed
   * to face the currently visible AprilTag.
   * @return drive speed rotation to face AprilTag
   */
  public static double calcSpeedFaceTag() {
    // Tx is the offset from the center of the camera (2d not 3d) 
    double tx = LimelightHelpers.getTX("limelight"); 

    double rotationSpeed;

    if (Math.abs(tx) <= Constants.ShooterConstants.crossPoint) {
      // Between +- where the functions cross, use linear
      rotationSpeed = -tx / Constants.ShooterConstants.linearDivider;
    } else { 
      // Use cubic since it's outside the inner range
      rotationSpeed = -Math.pow(tx, 3) / Constants.ShooterConstants.cubicDivider;
    }

    rotationSpeed *= Constants.ShooterConstants.speedScale;

    System.out.println(tx);
    System.out.println(rotationSpeed);

    return clampNum(rotationSpeed, -1, 1);
  }
}

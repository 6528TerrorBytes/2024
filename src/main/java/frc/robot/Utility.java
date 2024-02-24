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

        return (
            (RobotContainer.teamColor == DriverStation.Alliance.Blue && id == 7) ||
            (RobotContainer.teamColor == DriverStation.Alliance.Red && id == 4)
        );
    }

    public static boolean aprilTagInView() {
        return LimelightHelpers.getTV("limelight");
    }

    public static Pose3d aprilTagPos() {
        return LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
    }
}

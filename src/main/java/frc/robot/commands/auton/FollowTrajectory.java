package frc.robot.commands.auton;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utility;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// Uses the Holonomic Drive Controller at https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html
public class FollowTrajectory extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final Trajectory m_trajectory;

  private final double m_goalAngle;

  private HolonomicDriveController m_controller = new HolonomicDriveController(
    new PIDController(AutonConstants.kPXController, 0, 0), new PIDController(AutonConstants.kPYController, 0, 0),
    new ProfiledPIDController(AutonConstants.kPThetaController, 0, 0, AutonConstants.kThetaControllerConstraints)
  );

  private double m_timeStart;
  private boolean m_finished;

  // goalAngle is in degrees
  public FollowTrajectory(DriveSubsystem driveSubsystem, Trajectory trajectory, double goalAngle) {
    m_driveSubsystem = driveSubsystem;
    m_trajectory = trajectory;
    m_goalAngle = goalAngle;
    addRequirements(m_driveSubsystem);
  }

  @Override
  public void initialize() {
    m_timeStart = Utility.getTime();
    m_finished = false;
  }

  @Override
  public void execute() {
    double timeElapsed = Utility.getTime() - m_timeStart;
    if (timeElapsed > m_trajectory.getTotalTimeSeconds()) { 
      m_finished = true;
      return;
    }

    // Current point on the trajectory path
    Trajectory.State currentGoal = m_trajectory.sample(timeElapsed);

    // Calculate chassis speeds and then change to swerve module states
    ChassisSpeeds adjustedSpeeds = m_controller.calculate(
      m_driveSubsystem.getPose(), currentGoal, Rotation2d.fromDegrees(m_goalAngle)
    );
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(adjustedSpeeds);

    m_driveSubsystem.setModuleStates(states);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.setX();
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
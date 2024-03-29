package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveSpeedUp extends Command {
  private final double normalSpeed = 1;
  private final double otherSpeed = 0.75;

  @Override
  public void initialize() {
    DriveSubsystem.speedMultiplier = otherSpeed;
  }

  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.speedMultiplier = normalSpeed;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
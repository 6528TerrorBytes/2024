package frc.robot.commands.auton.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.commands.teleop.StopNoteCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTilt;
import frc.robot.subsystems.StopNote;
import frc.robot.commands.auton.SpeedUpShooter;
import frc.robot.Constants;
import frc.robot.commands.AimShooter;
import frc.robot.commands.auton.FireShooter;

// Aims and shoots (only aims vertically, not horizontally)
public class AimAndShoot extends SequentialCommandGroup {
  public AimAndShoot(StopNote stopNote, ShooterSubsystem shooterSubsystem, ShooterTilt shooterTilt, ConveyerSubsystem conveyerSubsystem) {
    addCommands(
      new StopNoteCommand(stopNote, false), // Making sure the stop note goes up
      new ParallelCommandGroup(
        new SpeedUpShooter(shooterSubsystem, 1, Constants.AutonConstants.speedUpShooterSeconds),
        new AimShooter(shooterTilt, true) // Then aims the shooter up to the speaker
      ),
      new FireShooter(conveyerSubsystem, shooterSubsystem, Constants.AutonConstants.conveyerRunSeconds)
    );
  }
}
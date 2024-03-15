package frc.robot.commands.auton.groups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import frc.robot.commands.intake.ConveyerCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.teleop.StopNoteCommand;
import frc.robot.subsystems.ConveyerSubsystem;
import frc.robot.subsystems.DetectNote;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StopNote;

// Moves and intakes until it detects a note
public class MoveAndIntake extends ParallelDeadlineGroup {
  public MoveAndIntake(Command moveCommand, StopNote stopNote, DetectNote detectNote, ConveyerSubsystem conveyerSubsystem, IntakeSubsystem intakeSubsystem) {
    super( // Ends when the conveyer command ends, when a note has been detected
      new ConveyerCommand(conveyerSubsystem, detectNote, 1, true), // "Deadline" command (entire command ends when this ends)

      new IntakeCommand(intakeSubsystem, detectNote, 1), // Run intake
      new StopNoteCommand(stopNote, true), // Brings note stopper down
      moveCommand // Command to follow the trajectory
    );
  }
}
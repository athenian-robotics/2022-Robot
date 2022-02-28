package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


// 2 balls in
//
public class EnterShooterMode extends SequentialCommandGroup {
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public EnterShooterMode(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexerSubsystem, this.intakeSubsystem);
        addCommands(
                // go down a lil
                new PulseForTime(indexerSubsystem, intakeSubsystem, false).withTimeout(.1),
                new PulseForTime(indexerSubsystem, intakeSubsystem, true).withTimeout(.3)
        );
    }
}

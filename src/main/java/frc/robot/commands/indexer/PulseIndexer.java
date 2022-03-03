package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


//For use only with Command.withTimeout(int seconds) or Button.whenHeld(Command ...). WILL NOT STOP AUTOMATICALLY
public class PulseIndexer extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final boolean up;

    public PulseIndexer(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, boolean up) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem; this.up = up;
        addRequirements(this.indexerSubsystem, this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (up) {
            indexerSubsystem.startIndexer();
            intakeSubsystem.startIntakeToIndexerMotor();
        } else {
            indexerSubsystem.reverseIndexer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
        intakeSubsystem.stopIntakeToIndexerMotor();
    }
}

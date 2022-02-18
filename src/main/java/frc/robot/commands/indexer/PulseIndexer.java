package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;


//For use only with Command.withTimeout(int seconds) or Button.whenHeld(Command ...). WILL NOT STOP AUTOMATICALLY
public class PulseIndexer extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final boolean up;

    public PulseIndexer(IndexerSubsystem indexerSubsystem, boolean up) {
        this.indexerSubsystem = indexerSubsystem; this.up = up;
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void initialize() {
        if (up) {
            System.out.println("indexer up init");
            indexerSubsystem.startIndexer();
        } else {
            System.out.println("indexer down init");
            indexerSubsystem.reverseIndexer();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
    }
}

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;


public class PulseForTime extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private boolean up;

    public PulseForTime(IndexerSubsystem indexerSubsystem, boolean u) {
        this.indexerSubsystem = indexerSubsystem;
        this.up = u;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
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
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
    }
}

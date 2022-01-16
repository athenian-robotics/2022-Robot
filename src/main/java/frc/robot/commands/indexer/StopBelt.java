package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;


public class StopBelt extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final int delay;

    public StopBelt(IndexerSubsystem indexerSubsystem, int delay) {
        this.indexerSubsystem = indexerSubsystem;
        this.delay = delay;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - delay * 1000 > 0;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
    }
}

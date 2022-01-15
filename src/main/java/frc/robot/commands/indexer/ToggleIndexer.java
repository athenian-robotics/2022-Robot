package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;


public class ToggleIndexer extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;

    public ToggleIndexer(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void initialize() {
        indexerSubsystem.toggleIndexer();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

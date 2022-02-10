package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;


public class ToggleBelt extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;

    public ToggleBelt(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void initialize() {
        indexerSubsystem.toggleBelt();
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

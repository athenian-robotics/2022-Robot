package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;


//For use only with Button.whenHeld(Command ...)
public class RunIndexer extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;

    public RunIndexer(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void initialize() {
        indexerSubsystem.startIndexer();}

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();}
}

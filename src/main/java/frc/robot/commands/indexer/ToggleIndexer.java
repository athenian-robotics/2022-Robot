package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.IndexerSubsystem;


public class ToggleIndexer extends InstantCommand {
    // Setup indexer subsystem
    private final IndexerSubsystem indexerSubsystem;

    public ToggleIndexer(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void initialize() { // On initialize, toggle the indexer and its belt
        indexerSubsystem.setDefaultCommand(new ExampleCommand(indexerSubsystem));
        indexerSubsystem.toggleIndexer();
    }

    //DOES NOT WORK BUT IT'S THE RIGHT IDEA

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.setDefaultCommand(new QueueBalls(indexerSubsystem));
    }
}

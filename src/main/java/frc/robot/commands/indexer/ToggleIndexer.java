package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class ToggleIndexer extends InstantCommand {
    // Setup indexer subsystem
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ToggleIndexer(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.indexerSubsystem, this.indexerSubsystem);
    }

    @Override
    public void initialize() { // On initialize, toggle the indexer and its belt
        indexerSubsystem.toggleIndexer();
        if (indexerSubsystem.indexerRunning) {
            intakeSubsystem.startIntakeToIndexerMotor();
        } else intakeSubsystem.stopIntakeToIndexerMotor();
    }
}

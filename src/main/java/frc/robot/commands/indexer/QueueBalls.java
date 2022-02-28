package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class QueueBalls extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public QueueBalls(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.indexerSubsystem, this.intakeSubsystem);
    }

    @Override
    public void execute() {
        if (indexerSubsystem.ballPrimed()){
            indexerSubsystem.startIndexer();
            intakeSubsystem.startIntakeToIndexerMotor();
    }
        else {
            indexerSubsystem.stopIndexer();
            intakeSubsystem.stopIntakeToIndexerMotor();
        }
    }
}
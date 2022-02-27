package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;


public class QueueBalls extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private long startTime = 0;

    public QueueBalls(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void execute() {
        if (indexerSubsystem.ballPrimed()) {
            indexerSubsystem.startIndexer();
//            intakeSubsystem.startIntakeToIndexerMotor();
        }
//            startTime = RobotController.getFPGATime();
//            while (RobotController.getFPGATime() == startTime + 500000) ;
        else {
            indexerSubsystem.stopIndexer();
//            intakeSubsystem.stopIntakeToIndexerMotor();
        }
    }
}
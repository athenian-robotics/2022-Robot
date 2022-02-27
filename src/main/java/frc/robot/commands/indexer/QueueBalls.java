package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class QueueBalls extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private long startTime = 0;

    public QueueBalls(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.indexerSubsystem, this.intakeSubsystem);
    }

    @Override
    public void execute() {
        if (indexerSubsystem.ballPrimed()) {
            indexerSubsystem.startIndexer();
            intakeSubsystem.startIntakeToIndexerMotor();
            startTime = RobotController.getFPGATime();
            while (RobotController.getFPGATime() == startTime + 500000) ;
            indexerSubsystem.stopIndexer();
            intakeSubsystem.stopIntakeToIndexerMotor();
        }
    }
}
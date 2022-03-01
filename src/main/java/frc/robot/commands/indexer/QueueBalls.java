package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class QueueBalls extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private boolean ballQueued = false;
    private double queueStartTime = 0;

    public QueueBalls(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.indexerSubsystem, this.intakeSubsystem);
    }

    @Override
    public void execute() {
        if (intakeSubsystem.isExtended) { intakeSubsystem.startIntakeToIndexerMotor(); return; }
        if(!ballQueued) {
            if (indexerSubsystem.ballPrimed()) {
                intakeSubsystem.startIntakeToIndexerMotor();
                if (!ballQueued) queueStartTime = System.currentTimeMillis();
                ballQueued = true;
            } else {
                intakeSubsystem.stopIntakeToIndexerMotor();
            }
        } else if (System.currentTimeMillis() - queueStartTime > 1000) {
            intakeSubsystem.stopIntakeToIndexerMotor();
        }
    }
}
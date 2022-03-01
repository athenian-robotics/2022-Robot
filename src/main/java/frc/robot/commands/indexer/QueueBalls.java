package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        //ballQueued keeps track of if there's a ball and should get reset when a command that uses the indexer (presumably shooter code) is sheduled
        if(!ballQueued) {
            //Default state, trying to suck in balls
            if (indexerSubsystem.ballPrimed()) {
                //This only runs the first time we see a ball with the proximity sensor. start the intakeToIndexer wheels and flag that we see a ball
                intakeSubsystem.startIntakeToIndexerMotor();
                if (!ballQueued) queueStartTime = System.currentTimeMillis();
                ballQueued = true;
            } else if (!intakeSubsystem.isExtended) {
                //If we don't see a ball we should stop the intakeToIndexer motor unless the intake is running, in which case we'd like it to spin (at least until we see a ball)
                intakeSubsystem.stopIntakeToIndexerMotor();
            }
        } else if (System.currentTimeMillis() - queueStartTime > Constants.MechanismConstants.intakeToIndexerResidualIndexTimeMillis) {
            //Ball is under the intakeToIndexer wheels, and we'll wait a bit and then stop the wheels. A second ball will wait in the intake
            intakeSubsystem.stopIntakeToIndexerMotor();
        }
    }
}
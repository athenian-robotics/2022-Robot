package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;


//Assuming that one ball is resting at the top of the indexer, this command passes it to the running shooter in a reliable fashion
public class ShootTopBall extends SequentialCommandGroup {

    public ShootTopBall(IndexerSubsystem indexerSubsystem) {
        super(
                new PulseIndexer(indexerSubsystem, false).withTimeout(.1),
                new PulseIndexer(indexerSubsystem, true).withTimeout(.5)
        );
    }

    public ShootTopBall(IndexerSubsystem indexerSubsystem, double residualRunTime) {
        super(
                new PulseIndexer(indexerSubsystem, false).withTimeout(.1),
                new PulseIndexer(indexerSubsystem, true).withTimeout(residualRunTime)
        );
    }
}
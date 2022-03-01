package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


//Assuming that one ball is resting at the top of the indexer, this command passes it to the running shooter in a reliable fashion
public class ShootTopBall extends SequentialCommandGroup {

    public ShootTopBall(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        super(
                new PulseIndexer(intakeSubsystem, indexerSubsystem, false).withTimeout(.1),
                new PulseIndexer(intakeSubsystem, indexerSubsystem, true).withTimeout(.2)

        );
    }

    public ShootTopBall(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem, double residualRunTime) {
        super(
                new PulseIndexer(intakeSubsystem, indexerSubsystem, false).withTimeout(.1),
                new PulseIndexer(intakeSubsystem, indexerSubsystem, true).withTimeout(residualRunTime)
        );
    }
}
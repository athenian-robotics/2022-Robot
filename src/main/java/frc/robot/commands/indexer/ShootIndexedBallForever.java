package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;


//Must be ended manually!
public class ShootIndexedBallForever extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;

    public ShootIndexedBallForever(IndexerSubsystem indexerSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void initialize() {
        if (outtakeSubsystem.getWheelSpeed() < 1) this.cancel();
        indexerSubsystem.startIndexer();
        indexerSubsystem.ballIndexed = false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
    }
}

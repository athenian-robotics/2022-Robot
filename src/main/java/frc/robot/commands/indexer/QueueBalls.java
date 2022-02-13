package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.colorwheel.WheelColors;
import frc.robot.subsystems.IndexerSubsystem;


public class QueueBalls extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;

    public QueueBalls(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(this.indexerSubsystem);
    }

    @Override
    public void execute() {
        if (indexerSubsystem.ballPrimed()
                && (indexerSubsystem.primedBallColor() == WheelColors.BLUE
                || indexerSubsystem.primedBallColor() == WheelColors.RED))
            indexerSubsystem.startIndexer();
        else
            indexerSubsystem.stopIndexer();
    }
}

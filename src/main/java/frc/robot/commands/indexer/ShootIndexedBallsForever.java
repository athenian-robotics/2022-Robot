package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


//Must be ended manually!
public class ShootIndexedBallsForever extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ShootIndexedBallsForever(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.indexerSubsystem, this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.startIntake();
        intakeSubsystem.startIntakeToIndexerMotor();
        indexerSubsystem.startIndexer();
        indexerSubsystem.ballIndexed = false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopIntakeToIndexerMotor();
    }
}

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;


//Must be ended manually!
public class ShootIndexedBallForever extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;

    public ShootIndexedBallForever(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.indexerSubsystem, this.intakeSubsystem, this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        if (outtakeSubsystem.getWheelSpeed() < 1) this.cancel();
        //intakeSubsystem.startIntakeToIndexerMotor();
        indexerSubsystem.startIndexer();
        indexerSubsystem.ballIndexed = false;
    }

    @Override
    public void end(boolean interrupted) {
        //intakeSubsystem.stopIntakeToIndexerMotor();
        indexerSubsystem.stopIndexer();
    }
}

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class PulseIndexerForTime extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private boolean up;

    public PulseIndexerForTime(IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem, boolean u) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.up = u;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexerSubsystem, this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (up) {
            System.out.println("indexer up init");
            indexerSubsystem.startIndexer();
            intakeSubsystem.startIntakeToIndexerMotor();
        } else {
            System.out.println("indexer down init");
            indexerSubsystem.reverseIndexer();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
        intakeSubsystem.stopIntakeToIndexerMotor();
    }
}

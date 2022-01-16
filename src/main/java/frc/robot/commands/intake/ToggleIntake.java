package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.indexer.StartBelt;
import frc.robot.commands.indexer.StopBelt;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class ToggleIntake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    public ToggleIntake(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intakeSubsystem, this.indexerSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Toggling Intake.");
        intakeSubsystem.toggleIntake();
        if (intakeSubsystem.isRunning) {
            new StartBelt(indexerSubsystem).schedule();
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        if (!intakeSubsystem.isRunning)
            new StopBelt(indexerSubsystem, 5).schedule();
    }
}

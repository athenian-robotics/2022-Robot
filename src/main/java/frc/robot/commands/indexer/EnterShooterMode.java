package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;


// 2 balls in
//
public class EnterShooterMode extends SequentialCommandGroup {
    private final IndexerSubsystem indexerSubsystem;

    public EnterShooterMode(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.indexerSubsystem);
        addCommands(
                // go down a lil
                new PulseForTime(indexerSubsystem, false).withTimeout(.1),
                new PulseForTime(indexerSubsystem, true).withTimeout(.3)
        );
    }

    @Override
    public void initialize() {
        System.out.println("shooter mode");
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

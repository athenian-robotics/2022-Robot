package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;


// 2 balls in
public class EnterShooterMode extends SequentialCommandGroup {

    public EnterShooterMode(IndexerSubsystem indexerSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(indexerSubsystem);
        addCommands(
                // go down a lil
                new PulseForTime(indexerSubsystem, false).withTimeout(.1),
                new PulseForTime(indexerSubsystem, true).withTimeout(.3)
        );
    }
}
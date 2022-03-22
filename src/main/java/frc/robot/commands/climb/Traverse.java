package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;


public class Traverse extends SequentialCommandGroup {
    public Traverse(ClimberSubsystem climber) {
        addCommands(
                new SetBothTelescopePositions(climber, 0),
                new WinchSetPosition(climber, 0.08),
                new SetBothTelescopePositions(climber, 0.3),
                new WinchSetPosition(climber, 0.05),
                new WinchSetPosition(climber, 1),
                new SetBothTelescopeSpeed(climber, 1),
                new WinchSetPosition(climber, 0.724),
                new SetBothTelescopePositions(climber, 0.5),
                new ParallelCommandGroup(
                        new SetBothTelescopePositions(climber, 0),
                        new WinchSetPosition(climber, 0)
                )
        );
    }
}
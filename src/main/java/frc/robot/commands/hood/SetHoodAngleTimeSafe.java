package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.HoodSubsystem;

public class SetHoodAngleTimeSafe extends ParallelCommandGroup {
    public SetHoodAngleTimeSafe(HoodSubsystem hoodSubsystem, double angle) {
        super(
                new SetHoodAngle(hoodSubsystem, angle),
                new WaitCommand(
                        Math.max(
                                (Math.abs(hoodSubsystem.getHoodAngle() - hoodSubsystem.lastHoodAngle) / 6.8) - 1.4,
                                0)));
    }
}
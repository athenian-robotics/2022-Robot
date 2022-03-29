package frc.robot.commands.outtake;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.OuttakeSubsystem;

public class SetHoodAngleTimeSafe extends SequentialCommandGroup {
    public SetHoodAngleTimeSafe(OuttakeSubsystem outtakeSubsystem, double angle) {
        super(
                new SetHoodAngle(outtakeSubsystem, angle),
                new WaitCommand((Math.abs(outtakeSubsystem.getHoodAngle() - angle) / 6) - 1.4)
        );
    }
}
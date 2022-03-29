package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OuttakeSubsystem;


public class SetHoodAngle extends InstantCommand {
    private final OuttakeSubsystem outtakeSubsystem;
    private final double angle;

    public SetHoodAngle(OuttakeSubsystem outtakeSubsystem, double angle) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.angle = angle;
        addRequirements();
    }

    @Override
    public void initialize() {
        outtakeSubsystem.setHoodAngle(angle);
    }
}

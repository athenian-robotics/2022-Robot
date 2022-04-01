package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;


public class SetHoodAngle extends InstantCommand {
    private final ShooterSubsystem shooterSubsystem;
    private final double angle;

    public SetHoodAngle(ShooterSubsystem shooterSubsystem, double angle) {
        this.shooterSubsystem = shooterSubsystem; this.angle = angle;
        addRequirements();
    }

    @Override
    public void initialize() {
        shooterSubsystem.setHoodAngle(angle);
    }
}

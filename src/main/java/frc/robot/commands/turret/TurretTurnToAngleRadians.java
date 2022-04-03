package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;


public class TurretTurnToAngleRadians extends InstantCommand {
    private final TurretSubsystem turretSubsystem;
    private final double angle;

    public TurretTurnToAngleRadians(TurretSubsystem turretSubsystem, double angle) {
        this.turretSubsystem = turretSubsystem;
        this.angle = angle;
        addRequirements(this.turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.setTurretSetpointRadians(angle);
    }
}

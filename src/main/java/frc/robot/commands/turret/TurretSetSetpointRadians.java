package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.TurretSubsystem;

public class TurretSetSetpointRadians extends InstantCommand {
  private final TurretSubsystem turretSubsystem;
  private final double angle;

  public TurretSetSetpointRadians(TurretSubsystem turretSubsystem, double angle) {
    this.turretSubsystem = turretSubsystem;
    this.angle = angle;
    addRequirements();
  }

  @Override
  public void initialize() {
    turretSubsystem.setTurretSetpointRadians(angle);
  }
}

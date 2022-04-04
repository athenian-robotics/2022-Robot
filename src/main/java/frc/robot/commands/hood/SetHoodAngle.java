package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HoodSubsystem;

public class SetHoodAngle extends InstantCommand {
  private final HoodSubsystem hoodSubsystem;
  private final double angle;

  public SetHoodAngle(HoodSubsystem hoodSubsystem, double angle) {
    this.hoodSubsystem = hoodSubsystem;
    this.angle = angle;
    addRequirements(this.hoodSubsystem);
  }

  @Override
  public void initialize() {
    hoodSubsystem.setHoodAngle(angle);
  }
}

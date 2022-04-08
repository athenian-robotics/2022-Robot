package frc.robot.commands.portal;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PortalSubsystem;

public class DisablePortal extends InstantCommand {
  private final PortalSubsystem portalSubsystem;

  public DisablePortal(PortalSubsystem portalSubsystem) {
    this.portalSubsystem = portalSubsystem;
    addRequirements(this.portalSubsystem);
  }

  @Override
  public void initialize() {
    portalSubsystem.disable();
  }
}

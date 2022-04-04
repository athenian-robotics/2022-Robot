package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PortalSubsystem;

public class ToggleIntake extends InstantCommand {
  private final IntakeSubsystem intakeSubsystem;
  private final PortalSubsystem portalSubsystem;

  public ToggleIntake(IntakeSubsystem intakeSubsystem, PortalSubsystem portalSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.portalSubsystem = portalSubsystem;
    addRequirements(this.intakeSubsystem, this.portalSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.togglePneumatic();
    intakeSubsystem.toggleIntake(); // On initialize, toggle intake
    if (intakeSubsystem.isRunning) portalSubsystem.startPortal();
  }
}

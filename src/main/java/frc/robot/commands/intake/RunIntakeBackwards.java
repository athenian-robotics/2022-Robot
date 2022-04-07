package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PortalSubsystem;

public class RunIntakeBackwards extends CommandBase {
  private final IntakeSubsystem intakeSubsystem;
  private final PortalSubsystem portalSubsystem;

  public RunIntakeBackwards(IntakeSubsystem intakeSubsystem, PortalSubsystem portalSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.portalSubsystem = portalSubsystem;
    addRequirements(this.intakeSubsystem, this.portalSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.extendPneumatic();
    intakeSubsystem.startIntakeInverted();
    portalSubsystem.startPortalInverted();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.disable();
    portalSubsystem.disable();
  }
}

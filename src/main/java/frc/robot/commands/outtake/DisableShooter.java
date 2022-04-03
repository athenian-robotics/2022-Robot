package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class DisableShooter extends InstantCommand {
  private final ShooterSubsystem shooterSubsystem;

  public DisableShooter(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements();
  }

  @Override
  public void initialize() {
    shooterSubsystem.stopShooter();
  }
}

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterPower extends InstantCommand {
  private final ShooterSubsystem shooterSubsystem;
  private final double power;

  public SetShooterPower(ShooterSubsystem shooterSubsystem, double rps) {
    this.shooterSubsystem = shooterSubsystem;
    this.power = rps;
  }

  @Override
  public void initialize() {
    shooterSubsystem.setRPS(power);
  }
}

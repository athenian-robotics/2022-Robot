package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class EnableShooter extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;

  public EnableShooter(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(this.shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setRPS(shooterSubsystem.shuffleboardShooterPower);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(shooterSubsystem.getWheelSpeed() - shooterSubsystem.shuffleboardShooterPower)
        < 0.5;
  }
}

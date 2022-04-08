package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterPowerTimeSafe extends ParallelCommandGroup {
  public SetShooterPowerTimeSafe(ShooterSubsystem shooter, double power) {
    super(
        new SetShooterPower(shooter, power),
        new WaitUntilCommand(() -> Math.abs(shooter.getRPS() - power) < 0.2));
  }
}

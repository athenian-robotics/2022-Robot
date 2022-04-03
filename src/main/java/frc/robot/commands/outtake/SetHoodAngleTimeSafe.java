package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SetHoodAngleTimeSafe extends SequentialCommandGroup {
  public SetHoodAngleTimeSafe(ShooterSubsystem shooterSubsystem, double angle) {
    super(
        new SetHoodAngle(shooterSubsystem, angle),
        new WaitCommand((Math.abs(shooterSubsystem.getHoodAngle() - angle) / 6) - 1.4));
  }
}

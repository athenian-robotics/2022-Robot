package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterPowerWithLimelightTimeSafe extends ParallelCommandGroup {
  public SetShooterPowerWithLimelightTimeSafe(
      LimelightSubsystem limelight, ShooterSubsystem shooter, ShooterDataTable shooterDataTable) {
    super(
        new SetShooterPowerWithLimelight(shooterDataTable, limelight, shooter),
        new WaitUntilCommand(() -> Math.abs(shooter.getRPS() - shooter.getTargetRPS()) < 0.2));
  }
}

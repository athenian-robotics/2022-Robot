package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SetHoodAngleWithLimelightTimeSafe extends ParallelCommandGroup {
  public SetHoodAngleWithLimelightTimeSafe(
      ShooterDataTable shooterDataTable,
      LimelightSubsystem limelightSubsystem,
      HoodSubsystem hoodSubsystem) {
    super(
        new SetHoodAngleWithLimelight(shooterDataTable, limelightSubsystem, hoodSubsystem),
        new WaitCommand(
            Math.max(
                (Math.abs(hoodSubsystem.getHoodAngle() - hoodSubsystem.lastHoodAngle) / 6.8) - 1.4,
                0)));
  }
}

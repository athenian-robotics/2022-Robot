package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hood.SetHoodAngleWithLimelightTimeSafe;
import frc.robot.commands.indexer.ShootIndexedBallsForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.limelight.GuaranteeLimelightData;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.commands.shooter.DisableShooter;
import frc.robot.commands.shooter.SetShooterPowerWithLimelight;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class ShootTwo extends SequentialCommandGroup {
  public ShootTwo(
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem outtake,
      PortalSubsystem portal,
      HoodSubsystem hood,
      TurretSubsystem turret,
      LimelightSubsystem limelight,
      ShooterDataTable shooterDataTable) {
    addCommands(
        // Prepare
        new DisableIntake(intake),
        // Shoot while letting Teddy drive
        // Find target while manually turning turret
        new GuaranteeLimelightData(limelight),
        // Shoot Balls
        // Set shooter power, angle, and offset while turning to goal
            new ParallelCommandGroup(
        new GuaranteeLimelightDataEquals(
            limelight,
            LimelightDataType.HORIZONTAL_OFFSET,
            0,
            turret.currentTurretToleranceRadians),
        new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
        new SetHoodAngleWithLimelightTimeSafe(shooterDataTable, limelight, hood)),
        new ShootIndexedBallsForever(indexer, intake, portal).withTimeout(1.7),
        // 1.94 <= distance <= 5 because of shooterDataTable minimums and maximums
        // Return to teleop
        new DisableShooter(outtake));
  }
}

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.hood.SetHoodAngleWithLimelightTimeSafe;
import frc.robot.commands.indexer.ShootIndexedBallsForever;
import frc.robot.commands.shooter.SetShooterPowerWithLimelight;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class ShootTwo extends SequentialCommandGroup {
  public ShootTwo(
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      PortalSubsystem portal,
      HoodSubsystem hood,
      TurretSubsystem turret,
      LimelightSubsystem limelight,
      ShooterDataTable shooterDataTable) {
    addCommands(
        // Prepare
        new InstantCommand(intake::disable, intake),
        // Set shooter power, angle, and offset while turning to goal
        new ParallelCommandGroup(
            new WaitUntilCommand(() -> turret.atGoal(Math.toRadians(4.2 / limelight.distance)))
                .withTimeout(2),
            new SetShooterPowerWithLimelight(shooterDataTable, limelight, shooter),
            new SetHoodAngleWithLimelightTimeSafe(shooterDataTable, limelight, hood)),
        new ShootIndexedBallsForever(indexer, intake, portal).withTimeout(1.7));
  }
}

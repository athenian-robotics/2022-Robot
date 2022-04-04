package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootIndexedBallForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.commands.portal.PulsePortal;
import frc.robot.commands.shooter.SetShooterPowerWithLimelight;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

// ARCHIVED
public class ShootOne extends SequentialCommandGroup {
  public ShootOne(
      ClimberSubsystem climber,
      DrivetrainSubsystem drivetrain,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem outtake,
      PortalSubsystem portal,
      LimelightSubsystem limelight,
      ShooterDataTable shooterDataTable) {
    addCommands(
        // Prepare
        new DisableDrivetrain(drivetrain),
        new DisableIntake(intake),
        // Align to shoot
        new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
        new GuaranteeLimelightDataEquals(limelight),
        new GuaranteeLimelightDataEquals(limelight),
        new ParallelCommandGroup(
            new PulsePortal(portal, 0.5), new ShootIndexedBallForever(indexer).withTimeout(2)),
        // Return to teleop
        new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake));
  }
}

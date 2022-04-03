package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.indexer.ShootIndexedBallsForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

// FOR AUTOS
public class ShootTwoWithoutTurret extends SequentialCommandGroup {
  public ShootTwoWithoutTurret(
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
        new DisableIntake(intake),
        // Align to shoot,
        new ParallelCommandGroup(
            new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
            new SetHoodAngleWithLimelightTimeSafe(shooterDataTable, limelight, outtake)),
        // Shoot Balls
        new ShootIndexedBallsForever(indexer, intake, portal).withTimeout(1.75),
        // Return to teleop
        new ParallelCommandGroup(
            new DisableShooter(outtake),
            new SetHoodAngle(outtake, Constants.MechanismConstants.defaultHoodAngle)));
  }
}

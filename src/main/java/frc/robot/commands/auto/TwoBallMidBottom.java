package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.scoring.ShootTwo;
import frc.robot.commands.turret.TurretSetSetpointRadians;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class TwoBallMidBottom extends SequentialCommandGroup {
  public TwoBallMidBottom(
      DrivetrainSubsystem drivetrain,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      TurretSubsystem turret,
      HoodSubsystem hood,
      PortalSubsystem portal,
      LimelightSubsystem limelight,
      ShooterDataTable shooterDataTable) {
    addCommands(
        new ToggleIntake(intake, portal),
        new PPRamsete(drivetrain, "Auto Routine 5 Part 1", 4, .6, true),
        new TurretSetSetpointRadians(turret, -Math.PI),
        new ShootTwo(indexer, intake, shooter, portal, hood, turret, limelight, shooterDataTable));
  }
}

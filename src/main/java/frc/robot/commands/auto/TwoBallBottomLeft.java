package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.scoring.ShootTwo;
import frc.robot.commands.turret.TurretSetSetpointRadians;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class TwoBallBottomLeft extends SequentialCommandGroup {
  public TwoBallBottomLeft(
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
        new PPRamsete(drivetrain, "Auto Routine 1 Part 1", 1.5, 0.7, true),
        new TurretSetSetpointRadians(turret, -Math.PI),
        new WaitCommand(0.25),
        new ShootTwo(indexer, intake, shooter, portal, hood, turret, limelight, shooterDataTable));
  }
}

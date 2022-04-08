package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.commands.scoring.ShootTwoWithoutTurret;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class AutoRoutine3 extends SequentialCommandGroup {
  public AutoRoutine3(
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
        new AutoRoutine6(drivetrain, "Auto Routine 1 Part 1", 1.5, 0.7, true),
        new GuaranteeLimelightDataEquals(
            limelight,
            LimelightDataType.HORIZONTAL_OFFSET,
            0,
            turret.currentTurretToleranceRadians),
        new ShootTwoWithoutTurret(
            indexer, intake, shooter, hood, portal, limelight, shooterDataTable));
  }
}

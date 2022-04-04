package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoRoutine2Contents.AutoRoutine2Part1;
import frc.robot.commands.auto.AutoRoutine2Contents.AutoRoutine2Part2;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.scoring.ShootTwoWithoutTurret;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class AutoRoutine2 extends SequentialCommandGroup {
  public AutoRoutine2(
      DrivetrainSubsystem drivetrain,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      PortalSubsystem portal,
      LimelightSubsystem limelight,
      ShooterDataTable shooterDataTable) {
    addCommands(
        new SequentialCommandGroup(
            new ToggleIntake(intake, portal),
            new AutoRoutine2Part1(drivetrain), // drives up to ball
            new ToggleIntake(intake, portal),
            new ShootTwoWithoutTurret(
                indexer, intake, shooter, portal, limelight, shooterDataTable),
            new AutoRoutine2Part2(drivetrain) // drives to the right to get another ball
            ));
  }
}

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoRoutine1Contents.*;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.scoring.ShootTwoWithoutTurret;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class AutoRoutine4 extends SequentialCommandGroup {
  public AutoRoutine4(
      DrivetrainSubsystem drivetrain,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      PortalSubsystem portal,
      LimelightSubsystem limelight,
      ShooterDataTable shooterDataTable) {
    addCommands(
        new SequentialCommandGroup(
            new ToggleIntake(intake, portal),
            new AutoRoutine1Part1(drivetrain), // moves left to other ball
            // moves down to ball under starting position
            new ShootTwoWithoutTurret(
                indexer, intake, shooter, hood, portal, limelight, shooterDataTable),
            new AutoRoutine1Part2(drivetrain),
            new ToggleIntake(intake, portal),
            new AutoRoutine1Part25(drivetrain), // moves left to other ball
            // drives to human player terminal
            new ToggleIntake(intake, portal),
            new AutoRoutine1Part4(drivetrain), // drives back closer to the goal
            new ShootTwoWithoutTurret(
                indexer, intake, shooter, hood, portal, limelight, shooterDataTable)));
  }
}

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
      HoodSubsystem hood,
      PortalSubsystem portal,
      LimelightSubsystem limelight,
      ShooterDataTable shooterDataTable) {
    addCommands(
        new SequentialCommandGroup(
            new ToggleIntake(intake, portal),
            new AutoRoutine6(drivetrain, "Auto Routine 2 Part 3", 4, 1.5, true), // drives up to ball
            new ShootTwoWithoutTurret(
                indexer, intake, shooter, hood, portal, limelight, shooterDataTable),
            new AutoRoutine6(drivetrain, "Auto Routine 2 Part 2", 4, 1.5, false) // drives to the right to get another ball
            ));
  }
}

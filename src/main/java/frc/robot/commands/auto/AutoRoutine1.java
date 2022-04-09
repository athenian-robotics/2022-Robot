package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.scoring.ShootTwo;
import frc.robot.commands.turret.TurretSetSetpointRadians;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class AutoRoutine1 extends SequentialCommandGroup {
  public AutoRoutine1(
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
        new AutoRoutine6(
            drivetrain, "Auto Routine 1 Part 1", 4, 1.5, true), // moves left to other ball
        // moves down to ball under starting position
        new ToggleIntake(intake, portal),
        new TurretSetSetpointRadians(turret, -Math.PI),
        new ShootTwo(indexer, intake, shooter, portal, hood, turret, limelight, shooterDataTable),
        new AutoRoutine6(drivetrain, "Auto Routine 1 Part 2", 4, 1.5, false), // turn back
        new ToggleIntake(intake, portal),
        new AutoRoutine6(
            drivetrain, "Auto Routine 1 Part 2.25", 4, 1.5, false), // intake other ball
            new TurretSetSetpointRadians(turret, -Math.PI),
        new ShootTwo(indexer, intake, shooter, portal, hood, turret, limelight, shooterDataTable)
            .withTimeout(1.5),
        new ToggleIntake(intake, portal),
        new AutoRoutine6(
            drivetrain, "Auto Routine 1 Part 3", 4, 1.5, false), // moves left to other ball
        // drives to human player terminal
        new WaitCommand(.75),
        new ToggleIntake(intake, portal),
        new AutoRoutine6(
            drivetrain, "Auto Routine 1 Part 4", 4, 2.25, false), // drives back closer to the goal
            new TurretSetSetpointRadians(turret, -Math.PI),
        new ShootTwo(indexer, intake, shooter, portal, hood, turret, limelight, shooterDataTable));
  }
}

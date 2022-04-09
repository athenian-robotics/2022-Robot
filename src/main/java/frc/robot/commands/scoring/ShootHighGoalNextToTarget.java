package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.hood.SetHoodAngle;
import frc.robot.commands.hood.SetHoodAngleTimeSafe;
import frc.robot.commands.indexer.ShootIndexedBallsForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.shooter.DisableShooter;
import frc.robot.commands.shooter.SetShooterPower;
import frc.robot.commands.turret.TurretSetSetpointRadians;
import frc.robot.subsystems.*;

public class ShootHighGoalNextToTarget extends SequentialCommandGroup {
  public ShootHighGoalNextToTarget(
      DrivetrainSubsystem drivetrain,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      PortalSubsystem portal,
      TurretSubsystem turret) {
    addCommands(
        new DisableDrivetrain(drivetrain),
        new DisableIntake(intake),
        // Align to shoot
        new SetShooterPower(shooter, 36),
        new ParallelCommandGroup(
            new SetHoodAngleTimeSafe(hood, 18.5), new TurretSetSetpointRadians(turret, 0)),
        new WaitCommand(1),
        // Shoot Balls
        new ShootIndexedBallsForever(indexer, intake, portal).withTimeout(2),
        // Return to teleop
        new ParallelCommandGroup(
            new DisableShooter(shooter),
            new SetHoodAngle(hood, Constants.MechanismConstants.defaultHoodAngle)));
  }
}

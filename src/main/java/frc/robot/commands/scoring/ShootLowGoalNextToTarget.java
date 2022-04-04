package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.hood.SetHoodAngle;
import frc.robot.commands.hood.SetHoodAngleTimeSafe;
import frc.robot.commands.indexer.ShootIndexedBallsForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.shooter.DisableShooter;
import frc.robot.commands.shooter.SetShooterPower;
import frc.robot.subsystems.*;

public class ShootLowGoalNextToTarget extends SequentialCommandGroup {
  public ShootLowGoalNextToTarget(
      DrivetrainSubsystem drivetrain,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      PortalSubsystem portal) {
    // Prepare
    addCommands(
        new DisableDrivetrain(drivetrain),
        new DisableIntake(intake),
        // Align to shoot
        new SetShooterPower(shooter, 17.5),
        new SetHoodAngleTimeSafe(hood, Constants.MechanismConstants.defaultHoodAngle),
        // Shoot Balls
        new ShootIndexedBallsForever(indexer, intake, portal).withTimeout(2),
        // Return to teleop
        new ParallelCommandGroup(
            new DisableShooter(shooter),
            new SetHoodAngle(hood, Constants.MechanismConstants.defaultHoodAngle)));
  }
}

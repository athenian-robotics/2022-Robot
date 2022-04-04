package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootIndexedBallForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.intake.RunIntakeWithoutPneumatics;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.commands.portal.PulsePortal;
import frc.robot.commands.shooter.DisableShooter;
import frc.robot.commands.shooter.SetShooterPower;
import frc.robot.commands.shooter.SetShooterPowerWithLimelight;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class ShootBalls extends SequentialCommandGroup {
  public ShootBalls(
      DrivetrainSubsystem drivetrain,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      PortalSubsystem portal,
      LimelightSubsystem limelight,
      ShooterDataTable shooterDataTable) {
    addCommands(
        // Prepare
        new DisableDrivetrain(drivetrain),
        new DisableIntake(intake),
        // Find target while manually turning turret
        new GuaranteeLimelightDataEquals(limelight).withTimeout(1.5),
        // Set power and hood angle
        new SetShooterPowerWithLimelight(shooterDataTable, limelight, shooter),
        // Align and shoot ~ball~ while turning turret
        new ConditionalCommand(
            new SequentialCommandGroup(
                new GuaranteeLimelightDataEquals(limelight),
                new ParallelCommandGroup(
                    new PulsePortal(portal).withTimeout(0.3),
                    new ShootIndexedBallForever(indexer).withTimeout(1.5)),
                new RunIntakeWithoutPneumatics(intake, portal).withTimeout(0.75)),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SetShooterPower(shooter, 15),
                    new ParallelCommandGroup(
                        new PulsePortal(portal).withTimeout(0.3),
                        new ShootIndexedBallForever(indexer).withTimeout(1.5)),
                    new RunIntakeWithoutPneumatics(intake, portal).withTimeout(0.75)),
                new WaitCommand(0),
                portal::ballPrimed),
            portal::allianceBallPrimed),
        // Set shooter power and angle again
        new SetShooterPowerWithLimelight(shooterDataTable, limelight, shooter),
        // Align and shoot while aiming turret again
        new ConditionalCommand(
            new SequentialCommandGroup(
                new GuaranteeLimelightDataEquals(limelight),
                new ParallelCommandGroup(
                    new PulsePortal(portal).withTimeout(0.3),
                    new ShootIndexedBallForever(indexer).withTimeout(1)),
                new RunIntakeWithoutPneumatics(intake, portal).withTimeout(0.75)),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new SetShooterPower(shooter, 15),
                    new ParallelCommandGroup(
                        new PulsePortal(portal).withTimeout(0.3),
                        new ShootIndexedBallForever(indexer).withTimeout(1)),
                    new RunIntakeWithoutPneumatics(intake, portal).withTimeout(0.75)),
                new WaitCommand(0),
                portal::ballPrimed),
            portal::allianceBallPrimed),
        // Return to teleop
        new DisableShooter(shooter));
  }
}

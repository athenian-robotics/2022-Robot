package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.hood.SetHoodAngle;
import frc.robot.commands.indexer.ShootIndexedBallsForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.limelight.GuaranteeLimelightData;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.commands.shooter.DisableShooter;
import frc.robot.commands.shooter.SetShooterPowerWithLimelight;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class ShootTwo extends SequentialCommandGroup {
  public ShootTwo(
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      ShooterSubsystem outtake,
      PortalSubsystem portal,
      HoodSubsystem hood,
      LimelightSubsystem limelight,
      ShooterDataTable shooterDataTable) {
    addCommands(
        // Prepare
        new DisableIntake(intake),
        // Shoot while letting Teddy drive
        new SequentialCommandGroup(
            // Find target while manually turning turret
            new GuaranteeLimelightData(limelight),
            // Set shooter power, angle, and offset while turning to goal
            new GuaranteeLimelightDataEquals(limelight),
            new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
            // Shoot Balls
            new ShootIndexedBallsForever(indexer, intake, portal).withTimeout(1.75),
            // Return to teleop
            new DisableShooter(outtake),
            new SetHoodAngle(hood, Constants.MechanismConstants.defaultHoodAngle)));
  }
}

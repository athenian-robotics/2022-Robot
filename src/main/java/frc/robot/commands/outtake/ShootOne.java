package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.climb.SetBothTelescopePositions;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootIndexedBallForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.limelight.GuaranteeLimelightData;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;


public class ShootOne extends SequentialCommandGroup {
    public ShootOne(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer, IntakeSubsystem intake, OuttakeSubsystem outtake, LimelightSubsystem limelight, ShooterDataTable shooterDataTable) {
        if (climber.getLeftHeightPercent() > 0.3 || climber.getRightHeightPercent() > 0.3) this.cancel();
        addCommands(
                //Prepare
                new DisableDrivetrain(drivetrain),
                new SetBothTelescopePositions(climber, 0),
                new DisableIntake(intake),
                //Align to shoot
                new ParallelDeadlineGroup(new GuaranteeLimelightData(limelight), new ManualAdjustTurret(outtake)),
                new SetHoodAngleWithLimelight(shooterDataTable, limelight, outtake),
                new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
                new TurretTurnToGoalWithLimelight(limelight, outtake),
                //Shoot 1st
                new ShootIndexedBallForever(indexer, outtake).withTimeout(2.5),
                //Return to teleop
                new SetShooterPower(outtake, 0),
                new SetHoodAngle(outtake, Constants.MechanismConstants.defaultHoodAngle)
        );
    }
}
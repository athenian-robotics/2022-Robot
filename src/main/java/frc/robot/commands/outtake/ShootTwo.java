package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootIndexedBallForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.intake.PulseIntakeToIndexerMotor;
import frc.robot.commands.intake.RunIntakeWithoutPneumatics;
import frc.robot.commands.limelight.GuaranteeLimelightData;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;


public class ShootTwo extends SequentialCommandGroup {
    public ShootTwo(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer,
                    IntakeSubsystem intake, OuttakeSubsystem outtake, LimelightSubsystem limelight,
                    ShooterDataTable shooterDataTable) {
        if (climber.getLeftHeightPercent() > 0.3 || climber.getRightHeightPercent() > 0.3) this.cancel();
        addCommands(
                //Prepare
                new DisableDrivetrain(drivetrain),
                //new SetBothTelescopePositions(climber, 0),
                new DisableIntake(intake),
                //Align to shoot
                new ParallelDeadlineGroup(new GuaranteeLimelightData(limelight), new ManualAdjustTurret(outtake)),
                new SetHoodAngleWithLimelight(shooterDataTable, limelight, outtake),
                new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new GuaranteeLimelightDataEquals(limelight, LimelightDataType.HORIZONTAL_OFFSET, 0,
                                        0.5),
                                //Shoot 1st
                                new ParallelCommandGroup(
                                        new PulseIntakeToIndexerMotor(intake, 0.5),
                                        new ShootIndexedBallForever(indexer, outtake).withTimeout(1.75)
                                ),
                                //Index next ball (may or may not be there) TODO remove withTimeout() on
                                // RunIntakeWithoutPneumatics()
                                new RunIntakeWithoutPneumatics(intake, indexer).withTimeout(1.5),
                                //Shoot 2nd
                                new ParallelCommandGroup(
                                        new PulseIntakeToIndexerMotor(intake, 0.5),
                                        new ShootIndexedBallForever(indexer, outtake).withTimeout(1.75)
                                )
                        ),
                        new AlwaysTurretTurnToGoalWithLimelight(limelight, outtake)
                ),
                //Return to teleop
                new DisableShooter(outtake),
                new SetHoodAngle(outtake, Constants.MechanismConstants.defaultHoodAngle)
        );
    }
}
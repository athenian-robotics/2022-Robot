package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.*;
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


public class ShootBalls extends SequentialCommandGroup {
    public ShootBalls(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer,
                      IntakeSubsystem intake, OuttakeSubsystem outtake, LimelightSubsystem limelight,
                      ShooterDataTable shooterDataTable) {
        if (climber.getLeftHeightPercent() > 0.1 || climber.getRightHeightPercent() > 0.1) this.cancel();
        addCommands(
                //Prepare
                new DisableDrivetrain(drivetrain),
                new DisableIntake(intake),
                //Find target while manually turning turret
                new ParallelDeadlineGroup(new GuaranteeLimelightData(limelight), new ManualAdjustTurret(outtake)),
                //Set power and hood angle
                new ParallelCommandGroup(
                        new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
                        new SetHoodAngleWithLimelightTimeSafe(shooterDataTable, limelight, outtake)
                ),
                //Align and shoot ~ball~ while turning turret
                new ParallelDeadlineGroup(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new GuaranteeLimelightDataEquals(limelight,
                                                LimelightDataType.HORIZONTAL_OFFSET, 0,
                                                outtake.currentShooterToleranceDegrees),
                                        new ParallelCommandGroup(
                                                new PulseIntakeToIndexerMotor(intake).withTimeout(0.3),
                                                new ShootIndexedBallForever(indexer, outtake).withTimeout(1.5)
                                        ), new RunIntakeWithoutPneumatics(intake, indexer).withTimeout(0.75)
                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetShooterPower(outtake, 15),
                                                new ParallelCommandGroup(
                                                        new PulseIntakeToIndexerMotor(intake).withTimeout(0.3),
                                                        new ShootIndexedBallForever(indexer, outtake).withTimeout(1.5)
                                                ), new RunIntakeWithoutPneumatics(intake, indexer).withTimeout(0.75)
                                        ),
                                        new WaitCommand(0),
                                        indexer::ballPrimed
                                ),
                                indexer::allianceBallIndexed
                        ),
                        new AlwaysTurretTurnToGoalWithLimelight(limelight, outtake)
                ),
                //Set shooter power and angle again
                new ParallelCommandGroup(
                        new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
                        new SetHoodAngleWithLimelightTimeSafe(shooterDataTable, limelight, outtake)
                ),
                //Align and shoot while aiming turret again
                new ParallelDeadlineGroup(
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new GuaranteeLimelightDataEquals(limelight,
                                                LimelightDataType.HORIZONTAL_OFFSET, 0,
                                                outtake.currentShooterToleranceDegrees),
                                        new ParallelCommandGroup(
                                                new PulseIntakeToIndexerMotor(intake).withTimeout(0.3),
                                                new ShootIndexedBallForever(indexer, outtake).withTimeout(1)
                                        ), new RunIntakeWithoutPneumatics(intake, indexer).withTimeout(0.75)
                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new SetShooterPower(outtake, 15),
                                                new ParallelCommandGroup(
                                                        new PulseIntakeToIndexerMotor(intake).withTimeout(0.3),
                                                        new ShootIndexedBallForever(indexer, outtake).withTimeout(1)
                                                ), new RunIntakeWithoutPneumatics(intake, indexer).withTimeout(0.75)
                                        ),
                                        new WaitCommand(0),
                                        indexer::ballPrimed
                                ),
                                indexer::allianceBallIndexed
                        ),
                        new AlwaysTurretTurnToGoalWithLimelightAndSetHoodAngleOrManualControl(limelight, outtake,
                                shooterDataTable)
                ),
                //Return to teleop
                new DisableShooter(outtake),
                new SetHoodAngle(outtake, Constants.MechanismConstants.defaultHoodAngle)
        );
    }
}
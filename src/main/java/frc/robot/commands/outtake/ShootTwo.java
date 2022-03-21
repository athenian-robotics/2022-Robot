package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootIndexedBallForever;
import frc.robot.commands.indexer.ShootIndexedBallsForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.intake.PulseIntakeToIndexerMotor;
import frc.robot.commands.intake.RunIntakeWithoutPneumatics;
import frc.robot.commands.limelight.GuaranteeLimelightData;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;


public class ShootTwo extends SequentialCommandGroup {
    public ShootTwo(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer, IntakeSubsystem intake, OuttakeSubsystem outtake, LimelightSubsystem limelight, ShooterDataTable shooterDataTable) {
        if (climber.getLeftHeightPercent() > 0.1 || climber.getRightHeightPercent() > 0.1) this.cancel();
            addCommands(
                    //Prepare
                    new DisableDrivetrain(drivetrain),
                    new DisableIntake(intake),
                    //Align to shoot
                    new ParallelDeadlineGroup(new GuaranteeLimelightData(limelight), new ManualAdjustTurret(outtake)),
                    new ParallelCommandGroup(
                            new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
                            new SetHoodAngleWithLimelightTimeSafe(shooterDataTable, limelight, outtake)
                    ),
                    new AlwaysTurretTurnToGoalWithLimelight(limelight, outtake).withTimeout(0.75),
                    new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ParallelDeadlineGroup(
                                new GuaranteeLimelightDataEquals(limelight, LimelightDataType.HORIZONTAL_OFFSET, 0, 1),
                                new ManualAdjustTurret(outtake)
                                ),
                                //Shoot Balls
                                    new ShootIndexedBallsForever(indexer, intake).withTimeout(2)
                                ),
                        new AlwaysTurretTurnToGoalWithLimelight(limelight, outtake)
                    ),
                    //Return to teleop
                    new ParallelCommandGroup(
                            new DisableShooter(outtake),
                            new SetHoodAngle(outtake, Constants.MechanismConstants.defaultHoodAngle)
                    )
            );
    }
}
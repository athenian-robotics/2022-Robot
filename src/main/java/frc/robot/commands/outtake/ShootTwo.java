package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootIndexedBallsForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.limelight.GuaranteeLimelightData;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;


public class ShootTwo extends SequentialCommandGroup {
    public ShootTwo(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer,
                    IntakeSubsystem intake, ShooterSubsystem outtake, PortalSubsystem portal, LimelightSubsystem limelight,
                    ShooterDataTable shooterDataTable, XboxController xboxController) {
        addCommands(
                //Prepare
                new DisableDrivetrain(drivetrain),
                new DisableIntake(intake),
                //Shoot while letting Teddy drive
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                //Find target while manually turning turret
                                new ParallelDeadlineGroup(new GuaranteeLimelightData(limelight), new ManualAdjustTurret(outtake)),
                                //Set shooter power, angle, and offset while turning to goal
                                new ParallelDeadlineGroup(
                                        new SequentialCommandGroup(
                                                new GuaranteeLimelightDataEquals(limelight,
                                                        LimelightDataType.HORIZONTAL_OFFSET, 0,
                                                        outtake.currentShooterToleranceDegrees),
                                                new ParallelCommandGroup(
                                                        new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
                                                        new SetHoodAngleWithLimelightTimeSafe(shooterDataTable, limelight, outtake)
                                                ),
                                                //Shoot Balls
                                                new ShootIndexedBallsForever(indexer, intake, portal).withTimeout(1.75)
                                        ),
                                        new AlwaysTurretTurnToGoalWithLimelightAndSetHoodAngleOrManualControl(limelight, outtake, shooterDataTable)
                                ),
                                //Return to teleop
                                new DisableShooter(outtake),
                                new SetHoodAngle(outtake, Constants.MechanismConstants.defaultHoodAngle)
                        ), new ArcadeDrive(drivetrain, xboxController)
                )
        );
    }
}
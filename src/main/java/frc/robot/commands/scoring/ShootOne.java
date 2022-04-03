//package frc.robot.commands.outtake;
//
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.commands.drive.DisableDrivetrain;
//import frc.robot.commands.hood.SetHoodAngleWithLimelightTimeSafe;
//import frc.robot.commands.indexer.ShootIndexedBallForever;
//import frc.robot.commands.intake.DisableIntake;
//import frc.robot.commands.portal.PulsePortal;
//import frc.robot.commands.limelight.GuaranteeLimelightData;
//import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
//import frc.robot.commands.shooter.SetShooterPowerWithLimelight;
//import frc.robot.commands.turret.AlwaysTurretTurnToGoalWithLimelight;
//import frc.robot.commands.turret.ManualAdjustTurret;
//import frc.robot.lib.limelight.LimelightDataType;
//import frc.robot.lib.shooterData.ShooterDataTable;
//import frc.robot.subsystems.*;
//
//
////ARCHIVED
//public class ShootOne extends SequentialCommandGroup {
//    public ShootOne(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer,
//                    IntakeSubsystem intake, ShooterSubsystem outtake, PortalSubsystem portal, LimelightSubsystem limelight,
//                    ShooterDataTable shooterDataTable) {
//        addCommands(
//                //Prepare
//                new DisableDrivetrain(drivetrain),
//                new DisableIntake(intake),
//                //Align to shoot
//                new ParallelDeadlineGroup(new GuaranteeLimelightData(limelight), new ManualAdjustTurret(outtake)),
//                new ParallelCommandGroup(
//                        new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
//                        new SetHoodAngleWithLimelightTimeSafe(shooterDataTable, limelight, outtake)
//                ),
//                new AlwaysTurretTurnToGoalWithLimelight(limelight, outtake).withTimeout(0.75),
//                new ParallelDeadlineGroup(
//                        new SequentialCommandGroup(
//                                new GuaranteeLimelightDataEquals(limelight, LimelightDataType.HORIZONTAL_OFFSET, 0, outtake.currentShooterToleranceDegrees),
//                                //Shoot 1st
//                                new ParallelCommandGroup(
//                                        new PulsePortal(portal, 0.5),
//                                        new ShootIndexedBallForever(indexer).withTimeout(2)
//                                )
//                        ),
//                        new AlwaysTurretTurnToGoalWithLimelight(limelight, outtake)
//                ),
//                //Return to teleop
//                new ParallelCommandGroup(
//                        new SetShooterPowerWithLimelight(shooterDataTable, limelight, outtake),
//                        new SetHoodAngleWithLimelightTimeSafe(shooterDataTable, limelight, outtake)
//                )
//        );
//    }
//}
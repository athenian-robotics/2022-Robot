//package frc.robot.commands.auto;
//
//
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.commands.auto.AutoRoutine1Contents.AutoRoutine1Part1;
//import frc.robot.commands.intake.ToggleIntake;
//import frc.robot.commands.scoring.ShootTwoWithoutTurret;
//import frc.robot.lib.shooterData.ShooterDataTable;
//import frc.robot.subsystems.*;
//
//public class AutoRoutine3 extends SequentialCommandGroup {
//    public AutoRoutine3(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer,
//                        IntakeSubsystem intake, ShooterSubsystem outtake, PortalSubsystem portal, LimelightSubsystem limelight,
//                        ShooterDataTable shooterDataTable) {
//        addCommands(
//                new ToggleIntake(intake, portal),
//                new AutoRoutine1Part1(drivetrain),
//                new ToggleIntake(intake, portal),
//                new ShootTwoWithoutTurret(climber, drivetrain, indexer, intake, outtake, portal, limelight, shooterDataTable)
//        );
//    }
//}
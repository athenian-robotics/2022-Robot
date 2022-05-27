// package frc.robot.auto;
//
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.intake.ToggleIntake;
// import frc.robot.commands.scoring.ShootTwo;
// import frc.robot.commands.turret.TurretSetSetpointRadians;
// import frc.robot.lib.shooterData.ShooterDataTable;
// import frc.robot.subsystems.*;
//
// public class TwoBallTopLeft extends SequentialCommandGroup {
//  public TwoBallTopLeft(
//      DrivetrainSubsystem drivetrain,
//      IndexerSubsystem indexer,
//      IntakeSubsystem intake,
//      ShooterSubsystem shooter,
//      TurretSubsystem turret,
//      HoodSubsystem hood,
//      PortalSubsystem portal,
//      LimelightSubsystem limelight,
//      ShooterDataTable shooterDataTable) {
//    addCommands(
//        new ToggleIntake(intake, portal),
//        new PPRamsete(drivetrain, "Auto Routine 2 Part 3", 4, 1.5, true), // drives up to ball
//        new TurretSetSetpointRadians(turret, -Math.PI),
//        new ShootTwo(indexer, intake, shooter, portal, hood, turret, limelight, shooterDataTable),
//        new PPRamsete(
//            drivetrain,
//            "Auto Routine 2 Part 2",
//            4,
//            1.5,
//            false) // drives to the right to get another ball
//        );
//  }
// }

// package frc.robot.auto;
//
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Superstructure;
// import frc.robot.lib.shooterData.ShooterDataTable;
// import frc.robot.subsystems.*;
//
// public class FiveBallBottomLeft extends SequentialCommandGroup {
//  public FiveBallBottomLeft(
//      DrivetrainSubsystem drivetrain,
//      IndexerSubsystem indexer,
//      IntakeSubsystem intake,
//      ShooterSubsystem shooter,
//      TurretSubsystem turret,
//      HoodSubsystem hood,
//      PortalSubsystem portal,
//      LimelightSubsystem limelight,
//      ShooterDataTable shooterDataTable,
//      Superstructure superstructure) {
//    addCommands(
//        intake.suckExtended(),
//        new PPRamsete(
//            drivetrain, "Auto Routine 1 Part 1", 4, 1.5, true), // moves left to other ball
//        // moves down to ball under starting position
//        intake.idleRetracted(),
//        superstructure.shoot(),
//        new PPRamsete(drivetrain, "Auto Routine 1 Part 2", 4, 1.5, false), // turn back
//        intake.suckExtended(),
//        new PPRamsete(drivetrain, "Auto Routine 1 Part 2.25", 4, 1.5, false), // intake other ball
//        superstructure.shoot(),
//        intake.
//        new PPRamsete(
//            drivetrain, "Auto Routine 1 Part 3", 4, 1.5, false), // moves left to other ball
//        // drives to human player terminal
//        new WaitCommand(.75),
//        new ToggleIntake(intake, portal),
//        new PPRamsete(
//            drivetrain, "Auto Routine 1 Part 4", 4, 2.25, false), // drives back closer to the
// goal
//        new TurretSetSetpointRadians(turret, -Math.PI),
//        new ShootTwo(indexer, intake, shooter, portal, hood, turret, limelight,
// shooterDataTable));
//  }
// }

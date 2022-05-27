// package frc.robot.auto;
//
// import edu.wpi.first.wpilibj2.command.*;
// import frc.robot.Constants;
// import frc.robot.commands.hood.SetHoodAngle;
// import frc.robot.commands.hood.SetHoodAngleTimeSafe;
// import frc.robot.commands.indexer.ShootIndexedBallsForever;
// import frc.robot.commands.intake.ToggleIntake;
// import frc.robot.commands.scoring.ShootTwo;
// import frc.robot.commands.shooter.SetShooterPower;
// import frc.robot.commands.turret.TurretSetSetpointRadians;
// import frc.robot.lib.shooterData.ShooterDataTable;
// import frc.robot.subsystems.*;
//
// public class TwoBallTopLeftDefense extends SequentialCommandGroup {
//  public TwoBallTopLeftDefense(
//      DrivetrainSubsystem drivetrain,
//      IndexerSubsystem indexer,
//      IntakeSubsystem intake,
//      ShooterSubsystem shooter,
//      TurretSubsystem turret,
//      HoodSubsystem hood,
//      PortalSubsystem portal,
//      LimelightSubsystem limelight,
//      ShooterDataTable shooterDataTable) {
//    super(
//        new ToggleIntake(intake, portal),
//        new PPRamsete(drivetrain, "Auto 7.1", 4, 2, true),
//        new ShootTwo(indexer, intake, shooter, portal, hood, turret, limelight, shooterDataTable),
//        new ToggleIntake(intake, portal),
//        new PPRamsete(drivetrain, "Better 7.2", 4, 1.1, false),
//        new TurretSetSetpointRadians(turret, -Math.PI / 2),
//        new WaitCommand(0.25),
//        new InstantCommand(intake::disable, intake),
//        new SetShooterPower(shooter, 17.5),
//        new TurretSetSetpointRadians(turret, -Math.PI / 2),
//        new SetHoodAngleTimeSafe(hood, -Math.PI / 2),
//        new TurretSetSetpointRadians(turret, -Math.PI / 2),
//        new ShootIndexedBallsForever(indexer, intake, portal).withTimeout(2),
//        new ParallelCommandGroup(
//            new InstantCommand(shooter::disable, shooter),
//            new SetHoodAngle(hood, Constants.MechanismConstants.defaultHoodAngle)));
//  }
// }

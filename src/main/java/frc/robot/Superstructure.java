package frc.robot;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.lib.LTVDifferentialDriveController;
import frc.robot.subsystems.*;

public class Superstructure extends SubsystemBase {
  private static final InstantCommand IDIOT_COMMAND =
      new InstantCommand(() -> System.out.println("u are idiot"));
  private final HoodSubsystem hood;
  private final PortalSubsystem portal;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private final DrivetrainSubsystem drivetrain;
  private final LimelightSubsystem limelight;
  private final PoseEstimator poseEstimator;
  private final LTVDifferentialDriveController controller =
      new LTVDifferentialDriveController(
          LinearSystemId.identifyDrivetrainSystem(
              Constants.AutoConstants.kvVoltSecondsPerMeter,
              Constants.AutoConstants.kaVoltSecondsSquaredPerMeter,
              Constants.AutoConstants.kvAngular,
              Constants.AutoConstants.kaAngular,
              Constants.DriveConstants.trackWidth),
          Constants.DriveConstants.trackWidth,
          VecBuilder.fill(0.001, 0.001, 0.001, 0.5, 1), // these are reasonable
          VecBuilder.fill(12, 12), // max control effort
          Constants.looptime);

  public Superstructure(
      HoodSubsystem hood,
      PortalSubsystem portal,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      DrivetrainSubsystem drivetrain,
      LimelightSubsystem limelight,
      PoseEstimator poseEstimator) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.poseEstimator = poseEstimator;
    this.hood = hood;
    this.portal = portal;
    this.shooter = shooter;
    this.indexer = indexer;
  }

  @Override
  public void periodic() {}

  public CommandGroupBase datatableTesting() {
    return sequence(
        parallel(shooter.test(), shooter.waitUntilReady(), hood.test()),
        new WaitCommand(1.5),
        indexer.startIndexer(),
        new WaitCommand(1),
        portal.startPortal(),
        new WaitCommand(1.7), // TODO: tower shorter
        indexer.stopIndexer(),
        portal.stopPortal(),
        shooter.idle());
  }

  // shoot when next to hub
  public CommandGroupBase shootHub() {
    return sequence(
        parallel(shooter.shootHub(), shooter.waitUntilReady()).withTimeout(1.5),
        indexer.startIndexer(),
        portal.startPortal(),
        new WaitCommand(1.3),
        indexer.stopIndexer(),
        portal.stopPortal(),
        shooter.idle());
  }

  public Command shoot() {
    if (limelight.getDistance() < 1.53) return IDIOT_COMMAND;
    return sequence(
        // aim(),
        parallel(
                shooter.requestShot(),
                shooter.waitUntilReady(),
                hood.approachTarget(),
                new WaitCommand(1))
            .withTimeout(1.5),
        indexer.startIndexer(),
        new WaitCommand(.3),
        portal.startPortal(),
        new WaitCommand(1.7),
        indexer.stopIndexer(),
        portal.stopPortal(),
        shooter.idle());
  }

  private Command aim() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          if (!limelight.isTarget()) return;
          var target = limelight.getTrackedTarget().get();
          var wheelSpeeds = poseEstimator.getWheelSpeeds();
          var pose = poseEstimator.getPose();
          var wheelVoltages =
              controller.calculate(
                  pose,
                  wheelSpeeds.leftMetersPerSecond,
                  wheelSpeeds.rightMetersPerSecond,
                  new Pose2d(
                      pose.getTranslation(),
                      new Rotation2d(
                          Math.toDegrees(target.getYaw()) + pose.getRotation().getDegrees())),
                  0,
                  0);
          drivetrain.tankDriveVolts(wheelVoltages.left, wheelVoltages.right);
        },
        (x) -> {},
        () -> controller.atReference() || !limelight.isTarget() || limelight.getDistance() < 1.53,
        limelight,
        drivetrain,
        poseEstimator);
  }
}

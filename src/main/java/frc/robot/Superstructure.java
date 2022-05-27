package frc.robot;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class Superstructure extends SubsystemBase {
  private final DrivetrainSubsystem drivetrain;
  private final HoodSubsystem hood;
  private final IntakeSubsystem intake;
  private final PortalSubsystem portal;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;
  private TurretSubsystem turret;
  private State state;

  public Superstructure(
      DrivetrainSubsystem drivetrain,
      HoodSubsystem hood,
      IntakeSubsystem intake,
      PortalSubsystem portal,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      TurretSubsystem turret) {
    this.drivetrain = drivetrain;
    this.hood = hood;
    this.intake = intake;
    this.portal = portal;
    this.shooter = shooter;
    this.indexer = indexer;
    this.turret = turret;
    state = State.IDLE;
  }

  private enum State {
    SHOOTING,
    IDLE,
    HOLDING
  }

  @Override
  public void periodic() {}

  public CommandGroupBase datatableTesting() {
    return sequence(
        parallel(
                shooter.requestShot(),
                shooter.waitUntilReady(),
                turret.waitUntilSetpoint(),
                hood.approachTarget(),
                hood.waitUntilSetpoint())
            .withTimeout(3),
        parallel(indexer.startIndexer(), portal.startPortal()),
        new WaitCommand(1.7),
        indexer.stopIndexer(),
        portal.stopPortal(),
        shooter.idle());
  }

  public CommandGroupBase shoot() {
    return sequence(
        parallel(
                shooter.requestShot(),
                shooter.waitUntilReady(),
                turret.waitUntilSetpoint(),
                hood.approachTarget(),
                hood.waitUntilSetpoint())
            .withTimeout(3),
        parallel(indexer.startIndexer(), portal.startPortal()),
        new WaitCommand(1.7),
        indexer.stopIndexer(),
        portal.stopPortal(),
        shooter.idle());
  }
}

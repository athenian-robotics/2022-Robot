package frc.robot;

import static edu.wpi.first.wpilibj2.command.CommandGroupBase.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class Superstructure extends SubsystemBase {
  private final HoodSubsystem hood;
  private final PortalSubsystem portal;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;

  public Superstructure(
      HoodSubsystem hood,
      PortalSubsystem portal,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer) {
    this.hood = hood;
    this.portal = portal;
    this.shooter = shooter;
    this.indexer = indexer;
  }

  @Override
  public void periodic() {}

  public CommandGroupBase datatableTesting() {
    return sequence(
        parallel(
                shooter.requestShot(),
                shooter.waitUntilReady(),
                hood.approachTarget(),
                hood.waitUntilSetpoint())
            .withTimeout(3),
        parallel(indexer.startIndexer(), portal.startPortal()),
        new WaitCommand(1.7), // TODO: tower shorter
        indexer.stopIndexer(),
        portal.stopPortal(),
        shooter.idle());
  }

  // shoot when next to hub
  public CommandGroupBase shootHub() {
    return sequence(
        parallel(
                shooter.shootHub(),
                shooter.waitUntilReady(),
                hood.hoodHub(),
                hood.waitUntilSetpoint())
            .withTimeout(3),
        parallel(indexer.startIndexer(), portal.startPortal()),
        new WaitCommand(1.7), // TODO: tower shorter
        indexer.stopIndexer(),
        portal.stopPortal(),
        shooter.idle());
  }

  public CommandGroupBase shoot() {
    return sequence(
        parallel(
                shooter.requestShot(),
                shooter.waitUntilReady(),
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

package frc.robot.commands.portal;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PortalSubsystem;

public class QueueBalls extends CommandBase {
  private final PortalSubsystem portalSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private boolean ballQueued = false;
  private double queueStartTime = Integer.MAX_VALUE;

  public QueueBalls(PortalSubsystem portalSubsystem, IntakeSubsystem intakeSubsystem) {
    this.portalSubsystem = portalSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.portalSubsystem);
  }

  @Override
  public void initialize() {
    if (portalSubsystem.ballPrimed()) {
      queueStartTime = Integer.MIN_VALUE;
      ballQueued = true;
    } else {
      queueStartTime = Integer.MAX_VALUE;
      ballQueued = false;
    }
  }

  @Override
  public void execute() {
    // ballQueued keeps track of if there's a ball and should get reset when a command that uses the
    // indexer
    // (presumably shooter code) is sheduled
    if (!ballQueued) {
      // Default state, trying to suck in balls
      if (portalSubsystem.ballPrimed()) {
        // This only runs the first time we see a ball with the proximity sensor. start the
        // intakeToIndexer
        // wheels and flag that we see a ball
        portalSubsystem.startPortal();
        if (!ballQueued) queueStartTime = System.currentTimeMillis();
        ballQueued = true;
      } else if (!intakeSubsystem.isRunning) {
        // If we don't see a ball we should stop the portal motor unless the intake is running, in
        // which case we'd like it to spin (at least until we see a ball)
        portalSubsystem.stopPortal();
      }
    } else if (System.currentTimeMillis() - queueStartTime
        > Constants.MechanismConstants.intakeToIndexerResidualIndexTimeMillis) {
      // Ball is under the intakeToIndexer wheels, and we'll wait a bit and then stop the wheels. A
      // second ball
      // will wait in the intake
      portalSubsystem.stopPortal();
    }
  }
}

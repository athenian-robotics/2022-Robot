package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PortalSubsystem;

// Must be ended manually!
public class ShootIndexedBallsForever extends CommandBase {
  private final IndexerSubsystem indexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PortalSubsystem portalSubsystem;

  public ShootIndexedBallsForever(
      IndexerSubsystem indexerSubsystem,
      IntakeSubsystem intakeSubsystem,
      PortalSubsystem portalSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.portalSubsystem = portalSubsystem;
    addRequirements(this.indexerSubsystem, this.portalSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.startIntake();
    portalSubsystem.startPortal();
    indexerSubsystem.startIndexer();
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopIndexer();
    intakeSubsystem.stopIntake();
    portalSubsystem.stopPortal();
  }
}

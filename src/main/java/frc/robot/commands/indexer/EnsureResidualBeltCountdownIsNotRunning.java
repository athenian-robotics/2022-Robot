package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;


public class EnsureResidualBeltCountdownIsNotRunning extends InstantCommand { private final IndexerSubsystem indexerSubsystem; public EnsureResidualBeltCountdownIsNotRunning(IndexerSubsystem indexerSubsystem) {this.indexerSubsystem = indexerSubsystem; addRequirements(this.indexerSubsystem);}
    @Override public void initialize() {indexerSubsystem.residualBeltFlag = true;} }

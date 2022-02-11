package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IndexerSubsystem;


public class StartBelt extends InstantCommand { private final IndexerSubsystem indexerSubsystem; public StartBelt(IndexerSubsystem indexerSubsystem) {this.indexerSubsystem = indexerSubsystem; addRequirements(this.indexerSubsystem);}
    @Override public void initialize() {indexerSubsystem.startBelt();}}

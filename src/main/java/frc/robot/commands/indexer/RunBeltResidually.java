package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;


public class RunBeltResidually extends CommandBase {
    private final IndexerSubsystem indexerSubsystem;
    private final double startTime;

    public RunBeltResidually(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(this.indexerSubsystem);

        startTime = System.currentTimeMillis();
    }

    @Override
    public void initialize() {indexerSubsystem.startBelt();}

    @Override //Don't stop the belt if this command is meant to be interrupted. If we extend the intake after this command starts up we never want it to stop the belt until another residual is created when we disable intake
    public void end(boolean interrupted) {indexerSubsystem.residualBeltFlag = false; if (!interrupted) indexerSubsystem.stopBelt();}

    @Override
    public boolean isFinished() {return System.currentTimeMillis() - startTime > Constants.MechanismConstants.residualBeltRunTime*1000;}
}

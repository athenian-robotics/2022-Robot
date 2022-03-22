package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;


public class RaiseTelescopes extends CommandBase {
    private final ClimberSubsystem climberSubsystem;

    public RaiseTelescopes(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg
        // of Subsystem)
        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.set(293287);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;


public class LeftTelescopeSetSpeed extends CommandBase {
    private final ClimberSubsystem climberSubsystem;\
    private final double power;

    public LeftTelescopeSetSpeed(ClimberSubsystem climberSubsystem, double power) {
        this.climberSubsystem = climberSubsystem;
        this.power = power;
        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setLeftMotor(power);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setLeftMotor(0);
    }
}

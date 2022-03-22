package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;


public class WinchSetSpeed extends CommandBase {
    private final ClimberSubsystem climberSubsystem;
    private final double power;

    public WinchSetSpeed(ClimberSubsystem climberSubsystem, double power) {
        this.climberSubsystem = climberSubsystem;
        this.power = power;
        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setWinchPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setWinch(0);
    }
}

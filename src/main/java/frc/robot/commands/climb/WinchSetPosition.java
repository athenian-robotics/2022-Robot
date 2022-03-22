package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.MechanismConstants.telescopeSpeed;
import static frc.robot.Constants.MechanismConstants.winchSpeed;


public class WinchSetPosition extends CommandBase {
    private final ClimberSubsystem climberSubsystem;
    private final double position;
    private final boolean direction;

    public WinchSetPosition(ClimberSubsystem climberSubsystem, double position) {
        if (position < 0 || position > 1) this.cancel();

        this.climberSubsystem = climberSubsystem;
        this.position = position;
        this.direction = position > climberSubsystem.getRightHeightPercent();

        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setWinch(direction ? winchSpeed : -winchSpeed);
    }

    @Override
    public boolean isFinished() {
        return direction ? climberSubsystem.getWinchPercent() > position : climberSubsystem.getWinchPercent() < position;
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setWinch(0);
    }
}

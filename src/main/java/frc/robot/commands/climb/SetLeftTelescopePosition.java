package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.MechanismConstants.telescopeSpeed;


public class SetLeftTelescopePosition extends CommandBase {
    private final ClimberSubsystem climberSubsystem;
    private final double position;
    private final boolean direction;

    public SetLeftTelescopePosition(ClimberSubsystem climberSubsystem, double position) {
        if (position < 0 || position > 1) this.cancel();

        this.climberSubsystem = climberSubsystem;
        this.position = position;
        this.direction = position > climberSubsystem.getLeftHeightPercent();

        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setLeftMotor(direction ? telescopeSpeed : -telescopeSpeed);
    }

    @Override
    public boolean isFinished() {
        return direction ? climberSubsystem.getLeftHeightPercent() > position : climberSubsystem.getLeftHeightPercent() < position;
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setLeftMotor(0);
    }
}

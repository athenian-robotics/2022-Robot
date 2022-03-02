package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.MechanismConstants.telescopeSpeed;


public class SetBothTelescopePositions extends CommandBase {
    private final ClimberSubsystem climberSubsystem;
    private final double position;
    private final boolean leftDirection;
    private final boolean rightDirection;

    public SetBothTelescopePositions(ClimberSubsystem climberSubsystem, double position) {
        if (position < 0 || position > 1) this.cancel();

        this.climberSubsystem = climberSubsystem;
        this.position = position;
        this.leftDirection = position > climberSubsystem.getLeftHeightPercent();
        this.rightDirection = position > climberSubsystem.getRightHeightPercent();

        addRequirements(this.climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setLeftMotor(leftDirection ? telescopeSpeed : -telescopeSpeed);
        climberSubsystem.setRightMotor(rightDirection ? telescopeSpeed : -telescopeSpeed);
    }

    @Override
    public void execute() {
        if (Math.abs(climberSubsystem.getLeftHeightPercent() - position) < 0.03) climberSubsystem.setLeftMotor(0);
        if (Math.abs(climberSubsystem.getRightHeightPercent() - position) < 0.03) climberSubsystem.setRightMotor(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(climberSubsystem.getLeftHeightPercent() - position) < 0.03 && Math.abs(climberSubsystem.getRightHeightPercent() - position) < 0.03;
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setLeftMotor(0);
        climberSubsystem.setRightMotor(0);
    }
}

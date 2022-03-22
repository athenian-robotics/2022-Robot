package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.MechanismConstants.telescopeSpeed;


public class SetBothTelescopePositions extends CommandBase {
    private final ClimberSubsystem climber;
    private final double position;
    private final boolean leftDirection;
    private final boolean rightDirection;

    public SetBothTelescopePositions(ClimberSubsystem climber, double position) {
        if (position < 0 || position > 1) this.cancel();

        this.climber = climber;
        this.position = position;
        this.leftDirection = position > climber.getLeftHeightPercent();
        this.rightDirection = position > climber.getRightHeightPercent();

        addRequirements(this.climber);
    }

    @Override
    public void initialize() {
        climber.setLeftMotor(leftDirection ? telescopeSpeed : -telescopeSpeed);
        climber.setRightMotor(rightDirection ? telescopeSpeed : -telescopeSpeed);
    }

    @Override
    public void execute() {
        if (atGoal(climber.getLeftHeightPercent(), leftDirection)) climber.setLeftMotor(0);
        if (atGoal(climber.getRightHeightPercent(), rightDirection)) climber.setRightMotor(0);
    }

    @Override
    public boolean isFinished() {
        return atGoal(climber.getLeftHeightPercent(), leftDirection) && atGoal(climber.getRightHeightPercent(),
                rightDirection);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setLeftMotor(0);
        climber.setRightMotor(0);
    }

    //Util
    private boolean atGoal(double heightPercent, boolean direction) {
        return direction ? heightPercent > position : heightPercent < position;
    }
}

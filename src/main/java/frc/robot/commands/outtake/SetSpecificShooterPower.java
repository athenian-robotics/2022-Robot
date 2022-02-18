package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class SetSpecificShooterPower extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    private final double power;

    public SetSpecificShooterPower(OuttakeSubsystem outtakeSubsystem, double power) {
        this.outtakeSubsystem = outtakeSubsystem; this.power = power;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        outtakeSubsystem.setShooterPower(power);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(outtakeSubsystem.getFrontShooterAcceleration()) < 0.05 && Math.abs(outtakeSubsystem.getBackShooterAcceleration()) < 0.05;
    }
}

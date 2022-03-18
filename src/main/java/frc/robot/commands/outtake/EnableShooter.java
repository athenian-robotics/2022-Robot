package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class EnableShooter extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public EnableShooter(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        outtakeSubsystem.setRPS(outtakeSubsystem.shuffleboardShooterPower);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(outtakeSubsystem.getWheelSpeed() - outtakeSubsystem.shuffleboardShooterPower) < 0.5;
    }
}

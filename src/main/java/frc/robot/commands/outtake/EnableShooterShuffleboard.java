package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class EnableShooterShuffleboard extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public EnableShooterShuffleboard(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        outtakeSubsystem.setShooterPower(outtakeSubsystem.shuffleboardShooterPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

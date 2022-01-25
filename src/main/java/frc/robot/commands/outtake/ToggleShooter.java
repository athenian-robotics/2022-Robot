package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;

public class ToggleShooter extends CommandBase {
    private final OuttakeSubsystem outtake; // Define outtake subsystem

    public ToggleShooter(OuttakeSubsystem outtake) {
        this.outtake = outtake;
        addRequirements(outtake);
    }

    @Override
    public void initialize() {
        outtake.toggleShooter();
    } // On initialize/setup, toggle the shooter

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}

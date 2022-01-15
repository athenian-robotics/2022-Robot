package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;


public class ToggleIntakePneumatics extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public ToggleIntakePneumatics(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.togglePneumatic();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

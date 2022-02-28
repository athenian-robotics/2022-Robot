package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;


public class ToggleIntakePneumaticsWithWheelsOn extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public ToggleIntakePneumaticsWithWheelsOn(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (intakeSubsystem.isRunning) {
            intakeSubsystem.togglePneumatic();
            intakeSubsystem.startIntakeToIndexerMotor();
        } else {
            intakeSubsystem.startIntake();
            intakeSubsystem.togglePneumatic();
            intakeSubsystem.stopIntakeToIndexerMotor();
        }
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
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopIntakeToIndexerMotor();
    }
}

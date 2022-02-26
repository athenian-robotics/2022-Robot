package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;


public class RunIntakePneumaticsWithWheelsOn extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public RunIntakePneumaticsWithWheelsOn(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (intakeSubsystem.isRunning) {
            intakeSubsystem.togglePneumatic();
        } else {
            intakeSubsystem.startIntake();
            intakeSubsystem.togglePneumatic();
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

    }
}

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntake extends CommandBase {
    // Define necessary subsystems
    private final IntakeSubsystem intakeSubsystem;
    private final long start = 0;

    public ToggleIntake(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.togglePneumatic();
        Timer.delay(0.3);
        intakeSubsystem.toggleIntake(); // On initialize, toggle intake
        if (intakeSubsystem.isRunning) {
            intakeSubsystem.startIntakeToIndexerMotor();
        } else {
            intakeSubsystem.stopIntakeToIndexerMotor();
        }
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}

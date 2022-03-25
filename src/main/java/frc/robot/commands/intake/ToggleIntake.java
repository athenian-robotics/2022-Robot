package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntake extends InstantCommand {
    private final IntakeSubsystem intakeSubsystem;

    public ToggleIntake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.togglePneumatic();
        intakeSubsystem.toggleIntake(); // On initialize, toggle intake
        if (intakeSubsystem.isRunning) intakeSubsystem.startIntakeToIndexerMotor();
    }
}

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ToggleIntake extends CommandBase {
    // Define necessary subsystems
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final LEDSubsystem lEDSubsystem;

    public ToggleIntake(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, LEDSubsystem lEDSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.lEDSubsystem = lEDSubsystem;
        addRequirements(this.intakeSubsystem, this.indexerSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.toggleIntake(); // On initialize, toggle intake
        intakeSubsystem.togglePneumatic();
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

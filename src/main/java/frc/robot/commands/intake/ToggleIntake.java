package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.LED.SetAllLEDToBlack;
import frc.robot.commands.LED.SetLEDToIntakeColor;
import frc.robot.commands.indexer.StartBelt;
import frc.robot.commands.indexer.StopBelt;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;


public class ToggleIntake extends CommandBase {
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
        System.out.println("Toggling Intake.");
        intakeSubsystem.toggleIntake();
        if (intakeSubsystem.isRunning) {
            new StartBelt(indexerSubsystem).schedule();
            new SetLEDToIntakeColor(intakeSubsystem, lEDSubsystem, 255, 168, 0).schedule();
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
        if (!intakeSubsystem.isRunning)
            new StopBelt(indexerSubsystem, 5).schedule();
        new SetAllLEDToBlack(lEDSubsystem).schedule();
    }
}

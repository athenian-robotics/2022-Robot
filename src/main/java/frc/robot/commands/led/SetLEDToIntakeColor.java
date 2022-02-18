package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;


public class SetLEDToIntakeColor extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final LEDSubsystem lEDSubsystem;
    private int R;
    private int G;
    private int B;

    public SetLEDToIntakeColor(IntakeSubsystem intakeSubsystem, LEDSubsystem lEDSubsystem, int R, int G, int B) {
        this.intakeSubsystem = intakeSubsystem;
        this.lEDSubsystem = lEDSubsystem;
        this.R = R;
        this.G = G;
        this.B = B;
        addRequirements(this.intakeSubsystem, this.lEDSubsystem);
    }

    @Override
    public void initialize() {
        this.lEDSubsystem.setAllColor(R, G, B);
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

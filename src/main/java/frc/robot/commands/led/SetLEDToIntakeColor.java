package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;


public class SetLEDToIntakeColor extends CommandBase {
    private final LEDSubsystem lEDSubsystem;
    private final int R;
    private final int G;
    private final int B;

    public SetLEDToIntakeColor(IntakeSubsystem intakeSubsystem, LEDSubsystem lEDSubsystem, int R, int G, int B) {
        this.lEDSubsystem = lEDSubsystem;
        this.R = R;
        this.G = G;
        this.B = B;
        addRequirements(intakeSubsystem, this.lEDSubsystem);
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

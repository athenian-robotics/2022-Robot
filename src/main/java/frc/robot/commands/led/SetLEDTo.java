package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;


public class SetLEDTo extends InstantCommand {
    private final LEDSubsystem lEDSubsystem;
    private final int R;
    private final int G;
    private final int B;

    public SetLEDTo(LEDSubsystem lEDSubsystem, int R, int G, int B) {
        this.lEDSubsystem = lEDSubsystem;
        this.R = R;
        this.G = G;
        this.B = B;
        addRequirements(this.lEDSubsystem);
    }

    @Override
    public void initialize() {
        this.lEDSubsystem.setAllColor(R, G, B);
    }
}

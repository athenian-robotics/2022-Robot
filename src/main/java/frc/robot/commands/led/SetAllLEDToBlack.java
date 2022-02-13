package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;


public class SetAllLEDToBlack extends CommandBase {
    private final LEDSubsystem lEDSubsystem;

    public SetAllLEDToBlack(LEDSubsystem lEDSubsystem) {
        this.lEDSubsystem = lEDSubsystem;
        addRequirements(this.lEDSubsystem);
    }

    @Override
    public void initialize() {
        lEDSubsystem.setAllColor(0, 0, 0);
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

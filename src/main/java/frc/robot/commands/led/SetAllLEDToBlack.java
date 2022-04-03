package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDSubsystem;

public class SetAllLEDToBlack extends InstantCommand {
  private final LEDSubsystem lEDSubsystem;

  public SetAllLEDToBlack(LEDSubsystem lEDSubsystem) {
    this.lEDSubsystem = lEDSubsystem;
    addRequirements(this.lEDSubsystem);
  }

  @Override
  public void initialize() {
    lEDSubsystem.setAllColor(0, 0, 0);
  }
}

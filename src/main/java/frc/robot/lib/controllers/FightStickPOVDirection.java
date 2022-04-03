package frc.robot.lib.controllers;

import static frc.robot.lib.controllers.FightStick.fightStickJoystick;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FightStickPOVDirection extends Trigger {
  private final FightStickInput.input direction;

  public FightStickPOVDirection(FightStickInput.input input) {
    this.direction = input;
  }

  @Override
  public boolean get() {
    return FightStickInput.getJoystickEnumValue(fightStickJoystick.getPOV()) == this.direction;
  }
}

package frc.robot.lib.controllers;


import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.lib.controllers.FightStick.fightStickJoystick;

public class FightStickPOVDirection extends Trigger {
    private FightStickInput.input direction;

    public FightStickPOVDirection(FightStickInput.input input) {
        this.direction = input;
    }

    @Override
    public boolean get() {
        return FightStickInput.getJoystickEnumValue(fightStickJoystick.getPOV()) == this.direction;
    }
}

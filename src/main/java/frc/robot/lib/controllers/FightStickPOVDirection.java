package frc.robot.lib.controllers;


import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.lib.controllers.FightStick.fightStickJoystick;

public class FightStickPOVDirection extends Trigger {
    private frc.robot.lib.controllers.FightStickInput.input direction;

    public FightStickPOVDirection(frc.robot.lib.controllers.FightStickInput.input input) {
        this.direction = input;
    }

    @Override
    public boolean get() {
        return frc.robot.lib.controllers.FightStickInput.getJoystickEnumValue(fightStickJoystick.getPOV()) == this.direction;
    }
}

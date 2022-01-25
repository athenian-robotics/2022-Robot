package frc.robot.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class FightStickAxisButton extends Trigger implements FightStickButton {

    private int axis;
    private frc.robot.lib.controllers.FightStickInput.input button;
    private Joystick fightStick;

    public FightStickAxisButton(Joystick fightStick, int axis, frc.robot.lib.controllers.FightStickInput.input button) {
        this.axis = axis;
        this.button = button;
        this.fightStick = fightStick;
    }

    @Override
    public boolean get() {
        return fightStick.getRawAxis(this.axis) > 0.5;
    }


    @Override
    public frc.robot.lib.controllers.FightStickInput.input getButtonInputType() {
        return this.button;
    }
}

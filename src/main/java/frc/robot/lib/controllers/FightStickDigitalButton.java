package frc.robot.lib.controllers;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class FightStickDigitalButton extends JoystickButton implements FightStickButton {

    private frc.robot.lib.controllers.FightStickInput.input button;

    public FightStickDigitalButton(Joystick stick, int buttonNumber, frc.robot.lib.controllers.FightStickInput.input button) {
        super(stick, buttonNumber);
        this.button = button;
    }

    @Override
    public frc.robot.lib.controllers.FightStickInput.input getButtonInputType() {
        return this.button;
    }

}
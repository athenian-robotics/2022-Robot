package frc.robot.lib.controllers;

/**
 * Simple, quick interface to ensure type similarities
 */

public interface FightStickButton {

    frc.robot.lib.controllers.FightStickInput.input getButtonInputType();

    boolean get();

}

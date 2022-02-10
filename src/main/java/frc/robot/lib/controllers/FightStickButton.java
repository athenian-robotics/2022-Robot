package frc.robot.lib.controllers;

/**
 * Simple, quick interface to ensure type similarities
 */

public interface FightStickButton {

    FightStickInput.input getButtonInputType();

    boolean get();

}

package frc.robot.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class FightStick {
    public static Joystick fightStickJoystick = new Joystick(Constants.OIConstants.fightStickPort);
    public static FightStickDigitalButton fightStickA = new FightStickDigitalButton(fightStickJoystick, 1, FightStickInput.input.medKick);
    public static FightStickDigitalButton fightStickB = new FightStickDigitalButton(fightStickJoystick, 2, FightStickInput.input.heavyKick);
    public static FightStickDigitalButton fightStickX = new FightStickDigitalButton(fightStickJoystick, 3, FightStickInput.input.medPunch);
    public static FightStickDigitalButton fightStickY = new FightStickDigitalButton(fightStickJoystick, 4, FightStickInput.input.heavyPunch);
    public static FightStickDigitalButton fightStickLB = new FightStickDigitalButton(fightStickJoystick, 5, FightStickInput.input.lightPunch);
    public static FightStickDigitalButton fightStickRB = new FightStickDigitalButton(fightStickJoystick, 6, FightStickInput.input.R1);
    public static FightStickDigitalButton fightStickShare = new FightStickDigitalButton(fightStickJoystick, 7, FightStickInput.input.share);
    public static FightStickDigitalButton fightStickOption = new FightStickDigitalButton(fightStickJoystick, 8, FightStickInput.input.option);
    public static FightStickDigitalButton fightStickL3 = new FightStickDigitalButton(fightStickJoystick, 9, FightStickInput.input.L3);
    public static FightStickDigitalButton fightStickR3 = new FightStickDigitalButton(fightStickJoystick, 10, FightStickInput.input.R3);
    public static FightStickAxisButton fightStickLT = new FightStickAxisButton(fightStickJoystick, 2, FightStickInput.input.lightKick);
    public static FightStickAxisButton fightStickRT = new FightStickAxisButton(fightStickJoystick, 3, FightStickInput.input.R2);

    public static FightStickPOVDirection POVCenter = new FightStickPOVDirection(FightStickInput.input.POVcenter);
    public static FightStickPOVDirection POVUp = new FightStickPOVDirection(FightStickInput.input.POVtop);
    public static FightStickPOVDirection POVDown = new FightStickPOVDirection(FightStickInput.input.POVbot);
    public static FightStickPOVDirection POVRight = new FightStickPOVDirection(FightStickInput.input.POVright);
    public static FightStickPOVDirection POVLeft = new FightStickPOVDirection(FightStickInput.input.POVleft);

//    public static Joystick fightStickJoystick = new Joystick(Constants.OIConstants.fightStickPort);
//    public static FightStickDigitalButton fightStickA = new FightStickDigitalButton(fightStickJoystick, 2, FightStickInput.input.medKick);
//    public static FightStickDigitalButton fightStickB = new FightStickDigitalButton(fightStickJoystick, 3, FightStickInput.input.heavyKick);
//    public static FightStickDigitalButton fightStickX = new FightStickDigitalButton(fightStickJoystick, 1, FightStickInput.input.medPunch);
//    public static FightStickDigitalButton fightStickY = new FightStickDigitalButton(fightStickJoystick, 4, FightStickInput.input.heavyPunch);
//    public static FightStickDigitalButton fightStickLB = new FightStickDigitalButton(fightStickJoystick, 5, FightStickInput.input.lightPunch);
//    public static FightStickDigitalButton fightStickRB = new FightStickDigitalButton(fightStickJoystick, 6, FightStickInput.input.R1);
//    public static FightStickDigitalButton fightStickShare = new FightStickDigitalButton(fightStickJoystick, 9, FightStickInput.input.share);
//    public static FightStickDigitalButton fightStickOption = new FightStickDigitalButton(fightStickJoystick, 10, FightStickInput.input.option);
//    public static FightStickDigitalButton fightStickL3 = new FightStickDigitalButton(fightStickJoystick, 11, FightStickInput.input.L3);
//    public static FightStickDigitalButton fightStickR3 = new FightStickDigitalButton(fightStickJoystick, 12, FightStickInput.input.R3);
//    public static FightStickDigitalButton fightStickLT = new FightStickDigitalButton(fightStickJoystick, 7, FightStickInput.input.lightKick);
//    public static FightStickDigitalButton fightStickRT = new FightStickDigitalButton(fightStickJoystick, 8, FightStickInput.input.R2);
//    public static FightStickDigitalButton fightStickHome = new FightStickDigitalButton(fightStickJoystick, 13, FightStickInput.input.home);
//
//    public static FightStickPOVDirection POVCenter = new FightStickPOVDirection(FightStickInput.input.POVcenter);
//    public static FightStickPOVDirection POVUp = new FightStickPOVDirection(FightStickInput.input.POVtop);
//    public static FightStickPOVDirection POVDown = new FightStickPOVDirection(FightStickInput.input.POVbot);
//    public static FightStickPOVDirection POVRight = new FightStickPOVDirection(FightStickInput.input.POVright);
//    public static FightStickPOVDirection POVLeft = new FightStickPOVDirection(FightStickInput.input.POVleft);


}

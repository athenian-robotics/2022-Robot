package frc.robot.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class FightStick {
    public static Joystick fightStickJoystick = new Joystick(Constants.OIConstants.fightStickPort);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickA = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 1, frc.robot.lib.controllers.FightStickInput.input.medKick);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickB = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 2, frc.robot.lib.controllers.FightStickInput.input.heavyKick);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickX = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 3, frc.robot.lib.controllers.FightStickInput.input.medPunch);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickY = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 4, frc.robot.lib.controllers.FightStickInput.input.heavyPunch);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickLB = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 5, frc.robot.lib.controllers.FightStickInput.input.lightPunch);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickRB = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 6, frc.robot.lib.controllers.FightStickInput.input.R1);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickShare = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 7, frc.robot.lib.controllers.FightStickInput.input.share);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickOption = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 8, frc.robot.lib.controllers.FightStickInput.input.option);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickL3 = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 9, frc.robot.lib.controllers.FightStickInput.input.L3);
    public static frc.robot.lib.controllers.FightStickDigitalButton fightStickR3 = new frc.robot.lib.controllers.FightStickDigitalButton(fightStickJoystick, 10, frc.robot.lib.controllers.FightStickInput.input.R3);
    public static frc.robot.lib.controllers.FightStickAxisButton fightStickLT = new frc.robot.lib.controllers.FightStickAxisButton(fightStickJoystick, 2, frc.robot.lib.controllers.FightStickInput.input.lightKick);
    public static frc.robot.lib.controllers.FightStickAxisButton fightStickRT = new frc.robot.lib.controllers.FightStickAxisButton(fightStickJoystick, 3, frc.robot.lib.controllers.FightStickInput.input.R2);

    public static frc.robot.lib.controllers.FightStickPOVDirection POVCenter = new frc.robot.lib.controllers.FightStickPOVDirection(frc.robot.lib.controllers.FightStickInput.input.POVcenter);
    public static frc.robot.lib.controllers.FightStickPOVDirection POVUp = new frc.robot.lib.controllers.FightStickPOVDirection(frc.robot.lib.controllers.FightStickInput.input.POVtop);
    public static frc.robot.lib.controllers.FightStickPOVDirection POVDown = new frc.robot.lib.controllers.FightStickPOVDirection(frc.robot.lib.controllers.FightStickInput.input.POVbot);
    public static frc.robot.lib.controllers.FightStickPOVDirection POVRight = new frc.robot.lib.controllers.FightStickPOVDirection(frc.robot.lib.controllers.FightStickInput.input.POVright);
    public static frc.robot.lib.controllers.FightStickPOVDirection POVLeft = new frc.robot.lib.controllers.FightStickPOVDirection(frc.robot.lib.controllers.FightStickInput.input.POVleft);

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

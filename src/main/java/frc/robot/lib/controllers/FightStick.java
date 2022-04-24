package frc.robot.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class FightStick {
  public static final Joystick fightStickJoystick =
      new Joystick(Constants.OIConstants.fightStickPort);
  public static final FightStickDigitalButton fightStickA =
      new FightStickDigitalButton(fightStickJoystick, 1, FightStickInput.input.medKick);
  public static final FightStickDigitalButton fightStickB =
      new FightStickDigitalButton(fightStickJoystick, 2, FightStickInput.input.heavyKick);
  public static final FightStickDigitalButton fightStickX =
      new FightStickDigitalButton(fightStickJoystick, 3, FightStickInput.input.medPunch);
  public static final FightStickDigitalButton fightStickY =
      new FightStickDigitalButton(fightStickJoystick, 4, FightStickInput.input.heavyPunch);
  public static final FightStickDigitalButton fightStickLB =
      new FightStickDigitalButton(fightStickJoystick, 5, FightStickInput.input.lightPunch);
  public static final FightStickDigitalButton fightStickRB =
      new FightStickDigitalButton(fightStickJoystick, 6, FightStickInput.input.R1);
  public static final FightStickDigitalButton fightStickL3 =
      new FightStickDigitalButton(fightStickJoystick, 9, FightStickInput.input.L3);
  public static final FightStickDigitalButton fightStickR3 =
      new FightStickDigitalButton(fightStickJoystick, 10, FightStickInput.input.R3);
  public static final FightStickAxisButton fightStickLT =
      new FightStickAxisButton(fightStickJoystick, 2, FightStickInput.input.lightKick);
  public static final FightStickAxisButton fightStickRT =
      new FightStickAxisButton(fightStickJoystick, 3, FightStickInput.input.R2);
  public static FightStickDigitalButton fightStickShare =
      new FightStickDigitalButton(fightStickJoystick, 7, FightStickInput.input.share);
  public static FightStickDigitalButton fightStickOption =
      new FightStickDigitalButton(fightStickJoystick, 8, FightStickInput.input.option);
  public static FightStickPOVDirection POVCenter =
      new FightStickPOVDirection(FightStickInput.input.POVcenter);
  public static FightStickPOVDirection POVUp =
      new FightStickPOVDirection(FightStickInput.input.POVtop);
  public static FightStickPOVDirection POVDown =
      new FightStickPOVDirection(FightStickInput.input.POVbot);
  public static FightStickPOVDirection POVRight =
      new FightStickPOVDirection(FightStickInput.input.POVright);
  public static FightStickPOVDirection POVLeft =
      new FightStickPOVDirection(FightStickInput.input.POVleft);
}

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.HoodSubsystem;

public class ManualHoodAngleControl extends CommandBase {
  private final HoodSubsystem hoodSubsystem;

  public ManualHoodAngleControl(HoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(this.hoodSubsystem);
  }

  @Override
  public void execute() {
    if (FightStick.fightStickJoystick.getY() > 0.5)
      hoodSubsystem.setHoodAngle(hoodSubsystem.getHoodAngle() + 1);
    if (FightStick.fightStickJoystick.getY() < -0.5)
      hoodSubsystem.setHoodAngle(hoodSubsystem.getHoodAngle() - 1);
  }
}

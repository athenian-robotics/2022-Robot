package frc.robot.commands.turret;

import static frc.robot.Constants.MechanismConstants.slowTurretTurnSpeed;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretTurnToGoalWithLimelightOrManualAdjustTurret extends CommandBase {
  private final LimelightSubsystem limelightSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final LimelightDataLatch offsetLatch;

  public TurretTurnToGoalWithLimelightOrManualAdjustTurret(
      LimelightSubsystem limelightSubsystem, TurretSubsystem turretSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystem = turretSubsystem;
    offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
    addRequirements(this.limelightSubsystem, this.turretSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.addLatch(offsetLatch.reset());
    turretSubsystem.turnTurret(0);
  }

  @Override
  public void execute() {
    if (FightStick.fightStickJoystick.getX() < -0.5) {
      turretSubsystem.turnTurret(-turretTurnSpeed);
    } else if (FightStick.fightStickJoystick.getX() > 0.5) {
      turretSubsystem.turnTurret(turretTurnSpeed);
    } else if (FightStick.fightStickJoystick.getY() < -0.5) {
      turretSubsystem.turnTurret(-slowTurretTurnSpeed);
    } else if (FightStick.fightStickJoystick.getY() > 0.5) {
      turretSubsystem.turnTurret(slowTurretTurnSpeed);
    } else if (FightStick.fightStickShare.get()) {
      turretSubsystem.disable();
    } else {
      try {
        if (offsetLatch.unlocked()) {
          turretSubsystem.setTurretSetpointRadians(
              offsetLatch.open() + turretSubsystem.getTurretAngleRadians());
          throw new GoalNotFoundException(); // shortcut to latch reset  vvv  (since we've expended
          // it)
        }
      } catch (GoalNotFoundException e) {
        limelightSubsystem.addLatch(
            offsetLatch.reset()); // assuming we want to look for the goal forever
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.disable();
  }
}

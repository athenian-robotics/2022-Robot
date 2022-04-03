package frc.robot.commands.outtake;

import static frc.robot.Constants.MechanismConstants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AlwaysTurretTurnToGoalWithLimelightAndSetHoodAngleOrManualControl extends CommandBase {
  private final LimelightSubsystem limelightSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterDataTable shooterDataTable;
  private final LimelightDataLatch offsetLatch;
  private final LimelightDataLatch distanceLatch;

  public AlwaysTurretTurnToGoalWithLimelightAndSetHoodAngleOrManualControl(
      LimelightSubsystem limelightSubsystem,
      ShooterSubsystem shooterSubsystem,
      ShooterDataTable shooterDataTable) {
    this.limelightSubsystem = limelightSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterDataTable = shooterDataTable;
    offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
    distanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 5);
    addRequirements(this.shooterSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.addLatch(offsetLatch.reset());
    limelightSubsystem.addLatch(distanceLatch.reset());
  }

  @Override
  public void execute() {
    if (FightStick.fightStickJoystick.getX() < -0.5) { // TURRET ADJUSTMENT FALCON
      shooterSubsystem.turretRunning = false;
      shooterSubsystem.bangBangRunning = false;
      shooterSubsystem.turnTurret(-turretTurnSpeed);
    } else if (FightStick.fightStickJoystick.getX() > 0.5) {
      shooterSubsystem.turretRunning = false;
      shooterSubsystem.bangBangRunning = false;
      shooterSubsystem.turnTurret(turretTurnSpeed);
    } else if (FightStick.fightStickJoystick.getY() < -0.5) { // TURRET ADJUSTMENT FALCON
      shooterSubsystem.turretRunning = false;
      shooterSubsystem.bangBangRunning = false;
      shooterSubsystem.turnTurret(-slowTurretTurnSpeed);
    } else if (FightStick.fightStickJoystick.getY() > 0.5) {
      shooterSubsystem.turretRunning = false;
      shooterSubsystem.bangBangRunning = false;
      shooterSubsystem.turnTurret(slowTurretTurnSpeed);
    } else if (FightStick.fightStickShare.get()) {
      try {
        if (offsetLatch.unlocked()) {
          shooterSubsystem.setTurretPositionRadians(
              offsetLatch.open() + shooterSubsystem.getTurretAngleRadians());
          throw new GoalNotFoundException(); // shortcut to latch reset  vvv  (since we've expended
          // it)
        }
      } catch (GoalNotFoundException e) {
        limelightSubsystem.addLatch(
            offsetLatch.reset()); // assuming we want to look for the goal forever
      }
    } else {
      shooterSubsystem.stopTurret();
    }

    try {
      if (distanceLatch.unlocked()) {
        shooterSubsystem.setHoodAngle(shooterDataTable.getSpecs(distanceLatch.open()).getAngle());
        throw new GoalNotFoundException(); // shortcut to latch reset
      }
    } catch (GoalNotFoundException e) {
      limelightSubsystem.addLatch(distanceLatch.reset());
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopTurret();
    shooterSubsystem.setHoodAngle(defaultHoodAngle);
  }
}

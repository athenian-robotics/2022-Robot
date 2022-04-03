package frc.robot.commands.outtake;

import static frc.robot.Constants.MechanismConstants.*;
import static frc.robot.Constants.MechanismConstants.defaultHoodAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AlwaysTurretTurnToGoalWithOdometryAndSetHoodAngleOrManualControl extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterDataTable shooterDataTable;
  private final LimelightDataLatch distanceLatch;

  public AlwaysTurretTurnToGoalWithOdometryAndSetHoodAngleOrManualControl(
      DrivetrainSubsystem drivetrainSubsystem,
      LimelightSubsystem limelightSubsystem,
      ShooterSubsystem shooterSubsystem,
      ShooterDataTable shooterDataTable) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterDataTable = shooterDataTable;

    distanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 5);

    addRequirements(this.shooterSubsystem);
  }

  @Override
  public void initialize() {
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
      Transform2d goalVector =
          drivetrainSubsystem.getPose().minus(new Pose2d(8.25, 4.15, new Rotation2d(0)));
      shooterSubsystem.setTurretPositionRadians(
          Math.toRadians(drivetrainSubsystem.getGyroAngle())
              - Math.atan2(goalVector.getY(), goalVector.getX()));
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

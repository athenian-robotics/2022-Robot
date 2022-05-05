package frc.robot.commands.turret;

import static frc.robot.Constants.MechanismConstants.slowTurretTurnSpeed;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;
import static frc.robot.Constants.hub;
import static frc.robot.RobotContainer.poseEstimator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.TurretSubsystem;

/* Aims the turret with vision or poseestimator with manual override*/
public class AimTurret extends CommandBase {
  private final LimelightSubsystem limelightSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final PoseEstimator poseEstimator;

  public AimTurret(
      LimelightSubsystem limelightSubsystem,
      TurretSubsystem turretSubsystem,
      PoseEstimator poseEstimator) {
    this.poseEstimator = poseEstimator;
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystem = turretSubsystem;
    addRequirements(this.limelightSubsystem, this.turretSubsystem);
  }

  @Override
  public void initialize() {
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
      if (Timer.getFPGATimestamp() - limelightSubsystem.getLatestPose().timestamp < 0.07) {
        turretSubsystem.setTurretSetpointRadians(
            limelightSubsystem.getLatestPose().pose.getRotation().getRadians());
      } else {
        if (limelightSubsystem.timeSinceLastUpdate < 0.07) {
          turretSubsystem.setTurretSetpointRadians(limelightSubsystem.angleOffset);
        } else {
          turretSubsystem.setTurretSetpointRadians(
              hub.minus(poseEstimator.getPose()).getRotation().getRadians()
                  + Math.sin(
                      hub.minus(poseEstimator.getPose()).getTranslation().getY()
                          / hub.minus(poseEstimator.getPose()).getTranslation().getX()));
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.disable();
  }
}

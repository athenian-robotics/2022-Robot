package frc.robot.commands.outtake;

import static frc.robot.Constants.MechanismConstants.maximumTurretAngleRadians;
import static frc.robot.Constants.MechanismConstants.minimumTurretAngleRadians;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class TurretTurnToAngle extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  private final double angle;

  public TurretTurnToAngle(ShooterSubsystem shooterSubsystem, double angle) {
    this.shooterSubsystem = shooterSubsystem;
    this.angle = angle;
    addRequirements(this.shooterSubsystem);
  }

  @Override
  public void execute() {
    if (angle < minimumTurretAngleRadians && angle - shooterSubsystem.getTurretAngleRadians() < 0
        || angle > maximumTurretAngleRadians
            && angle - shooterSubsystem.getTurretAngleRadians() > 0)
      System.out.println(
          "Error! angle passed into TurretTurnToAngle " + this + " was out of bounds");
  }

  @Override
  public boolean isFinished() {
    return Math.abs(shooterSubsystem.getTurretAngleRadians() - Math.toRadians(angle))
        <= 0.075; // 0.5 position tolerance
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopTurret();
  }
}

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimTurret extends CommandBase {
  private final LimelightSubsystem limelightSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final LimelightDataLatch offsetLatch;

  public AutoAimTurret(LimelightSubsystem limelightSubsystem, TurretSubsystem turretSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystem = turretSubsystem;
    offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
    addRequirements(this.turretSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.addLatch(offsetLatch.reset());
  }

  @Override
  public void execute() {
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

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.disable();
  }
}

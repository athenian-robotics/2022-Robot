package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// If limelight data isn't found within a quarter-second, the default shooter power will be set.
public class SetShooterPowerWithLimelight extends CommandBase {
  private final ShooterDataTable shooterDataTable;
  private final LimelightSubsystem limelightSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightDataLatch distanceLatch;

  public SetShooterPowerWithLimelight(
      ShooterDataTable shooterDataTable,
      LimelightSubsystem limelightSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.shooterDataTable = shooterDataTable;
    this.limelightSubsystem = limelightSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.distanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE);

    addRequirements(this.shooterSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.addLatch(distanceLatch);
  }

  @Override
  public boolean isFinished() {
    try { // jump to end() as soon as we get data! If we receive none in time, we'll use the default
      // given by
      return distanceLatch.unlocked();
    } catch (GoalNotFoundException e) {
      return true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setRPS(shooterDataTable.getSpecs(distanceLatch.open()).getPower());
  }
}

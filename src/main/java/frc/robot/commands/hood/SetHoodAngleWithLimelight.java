package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// If limelight data isn't found within a quarter-second, the default hood angle will be set.
public class SetHoodAngleWithLimelight extends CommandBase {
  private final ShooterDataTable shooterDataTable;
  private final LimelightSubsystem limelightSubsystem;
  private final HoodSubsystem hoodSubsystem;

  public SetHoodAngleWithLimelight(
      ShooterDataTable shooterDataTable,
      LimelightSubsystem limelightSubsystem,
      HoodSubsystem hoodSubsystem) {
    this.shooterDataTable = shooterDataTable;
    this.limelightSubsystem = limelightSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(this.hoodSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    hoodSubsystem.setHoodAngle(shooterDataTable.getSpecs(limelightSubsystem.distance).getAngle());
  }
}

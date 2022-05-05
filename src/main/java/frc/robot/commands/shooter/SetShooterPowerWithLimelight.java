package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// If limelight data isn't found within a quarter-second, the default shooter power will be set.
public class SetShooterPowerWithLimelight extends CommandBase {
  private final ShooterDataTable shooterDataTable;
  private final LimelightSubsystem limelightSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public SetShooterPowerWithLimelight(
      ShooterDataTable shooterDataTable,
      LimelightSubsystem limelightSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.shooterDataTable = shooterDataTable;
    this.limelightSubsystem = limelightSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(this.shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setRPS(shooterDataTable.getSpecs(limelightSubsystem.distance).getPower());
  }
}

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class SetHoodAngleWithLimelightTimeSafe extends SequentialCommandGroup {
  public SetHoodAngleWithLimelightTimeSafe(
      ShooterDataTable shooterDataTable,
      LimelightSubsystem limelightSubsystem,
      HoodSubsystem hoodSubsystem,
      double currentAngle) {
    super(
        new SetHoodAngleWithLimelight(shooterDataTable, limelightSubsystem, hoodSubsystem),
        new WaitCommand((Math.abs(hoodSubsystem.getHoodAngle() - currentAngle) / 6.8) - 1.4));
  }
}

// While passing in the current angle seems dumb, you can see how it makes this so much cleaner.
// The issue is that hood.getHoodAngle() returns the setpoint, not the actual current angle.
// Although, there would be no clean way to get the setpoint if it did, so this works out

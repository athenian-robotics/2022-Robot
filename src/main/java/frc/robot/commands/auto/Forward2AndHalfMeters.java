package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Forward2AndHalfMeters extends SequentialCommandGroup {
  public Forward2AndHalfMeters(DrivetrainSubsystem drivetrain) {
    addCommands(new PPRamsete(drivetrain, "2.5 Meters Forward", 4, 1.5, true));
  }
}

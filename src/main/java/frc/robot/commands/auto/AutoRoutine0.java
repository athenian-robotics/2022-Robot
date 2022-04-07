package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoRoutine0 extends SequentialCommandGroup {
  public AutoRoutine0(DrivetrainSubsystem drivetrain) {
    addCommands(new AutoRoutine6(drivetrain, "2.5 Meters Forward", 4, 1.5, true));
  }
}

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoRoutine0Contents.AutoRoutine0Part1;
import frc.robot.subsystems.DrivetrainSubsystem;


public class AutoRoutine0 extends SequentialCommandGroup {
  public AutoRoutine0(DrivetrainSubsystem drivetrain) {
    addCommands(new AutoRoutine0Part1(drivetrain));
  }
}

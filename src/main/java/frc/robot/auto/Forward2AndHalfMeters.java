package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimator;

public class Forward2AndHalfMeters extends SequentialCommandGroup {
  public Forward2AndHalfMeters(DrivetrainSubsystem drivetrain, PoseEstimator poseEstimator) {
    addCommands(new PPRamsete(drivetrain, poseEstimator, "Auto Routine 1 Part 1", 1.5, 0.7, true));
  }
}

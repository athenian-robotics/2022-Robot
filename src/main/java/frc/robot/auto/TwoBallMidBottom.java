package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Superstructure;
import frc.robot.subsystems.*;

public class TwoBallMidBottom extends SequentialCommandGroup {
  public TwoBallMidBottom(
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      PoseEstimator poseEstimator,
      Superstructure superstructure) {
    addCommands(
        intake.suckExtended(),
        new PPRamsete(drivetrain, poseEstimator, "Auto Routine 5 Part 1", 4, .6, true),
        superstructure.shoot());
  }
}

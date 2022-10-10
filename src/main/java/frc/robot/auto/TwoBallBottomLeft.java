package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Superstructure;
import frc.robot.subsystems.*;

public class TwoBallBottomLeft extends SequentialCommandGroup {
  public TwoBallBottomLeft(
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      Superstructure superstructure,
      PoseEstimator poseEstimator) {
    addCommands(
        intake.suckExtended(),
        new PPRamsete(drivetrain, poseEstimator, "Auto Routine 1 Part 1", 1.5, 0.7, true),
        new WaitCommand(0.25),
        intake.idleRetracted(),
        superstructure.shootHub());
  }
}

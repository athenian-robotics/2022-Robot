package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Superstructure;
import frc.robot.subsystems.*;

public class TwoBallTopLeftDefense extends SequentialCommandGroup {
  public TwoBallTopLeftDefense(
      DrivetrainSubsystem drivetrain,
      IntakeSubsystem intake,
      PoseEstimator poseEstimator,
      Superstructure superStructure) {
    super(
        intake.suckExtended(),
        new PPRamsete(drivetrain, poseEstimator, "Auto 7.1", 4, 2, true),
        intake.idleRetracted(),
        new WaitCommand(0.25),
        superStructure.shoot(),
        new PPRamsete(drivetrain, poseEstimator, "Better 7.2", 4, 1.1, false),
        new WaitCommand(0.25));
  }
}

package frc.robot.auto;

import static frc.robot.Constants.AutoConstants.maxAutoAcceleration;
import static frc.robot.Constants.AutoConstants.maxAutoSpeed;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.LTVDifferentialDriveCommand;
import frc.robot.lib.LTVDifferentialDriveController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimator;

public class PPLTV extends CommandBase {
  private final LTVDifferentialDriveCommand LTVCommand;
  private final Trajectory trajectory;
  private final DrivetrainSubsystem drivetrain;
  private final PoseEstimator poseEstimator;
  private final boolean resetOdometry;

  public PPLTV(
      DrivetrainSubsystem drivetrain,
      PoseEstimator poseEstimator,
      String pathName,
      double maxVel,
      double maxAccel,
      boolean resetOdometry) {
    this.drivetrain = drivetrain;
    this.resetOdometry = resetOdometry;
    this.poseEstimator = poseEstimator;
    addRequirements(drivetrain);

    // load path from name
    trajectory =
        PathPlanner.loadPath(
            pathName, Math.min(maxVel, maxAutoSpeed), Math.min(maxAccel, maxAutoAcceleration));

    this.LTVCommand =
        new LTVDifferentialDriveCommand(
            trajectory,
            poseEstimator::getPose,
            new LTVDifferentialDriveController(
                LinearSystemId.identifyDrivetrainSystem(
                    Constants.AutoConstants.kvVoltSecondsPerMeter,
                    Constants.AutoConstants.kaVoltSecondsSquaredPerMeter,
                    Constants.AutoConstants.kvAngular,
                    Constants.AutoConstants.kaAngular,
                    Constants.DriveConstants.trackWidth),
                Constants.DriveConstants.trackWidth,
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1), // these are reasonable
                VecBuilder.fill(12, 12), // max control effort
                Constants.looptime),
            Constants.AutoConstants.kDriveKinematics,
            poseEstimator::getWheelSpeeds,
            drivetrain::tankDriveVolts,
            drivetrain);
  }

  public PPLTV(
      DrivetrainSubsystem drivetrain,
      PoseEstimator poseEstimator,
      Trajectory traj,
      double maxVel,
      double maxAccel) {
    this.drivetrain = drivetrain;
    this.poseEstimator = poseEstimator;
    this.trajectory = traj;
    this.resetOdometry = false;
    this.LTVCommand =
        new LTVDifferentialDriveCommand(
            traj,
            poseEstimator::getPose,
            new LTVDifferentialDriveController(
                LinearSystemId.identifyDrivetrainSystem(
                    Constants.AutoConstants.kvVoltSecondsPerMeter,
                    Constants.AutoConstants.kaVoltSecondsSquaredPerMeter,
                    Constants.AutoConstants.kvAngular,
                    Constants.AutoConstants.kaAngular,
                    Constants.DriveConstants.trackWidth),
                Constants.DriveConstants.trackWidth,
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1), // these are reasonable
                VecBuilder.fill(12, 12), // max control effort
                Constants.looptime),
            Constants.AutoConstants.kDriveKinematics,
            poseEstimator::getWheelSpeeds,
            drivetrain::tankDriveVolts,
            drivetrain);
  }

  @Override
  public void initialize() {
    if (resetOdometry) {
      poseEstimator.resetPose(trajectory.getInitialPose());
    }
    LTVCommand.initialize();
  }

  @Override
  public void execute() {
    LTVCommand.execute();
  }

  @Override
  public boolean isFinished() {
    return LTVCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    LTVCommand.end(interrupted);
    drivetrain.disable();
  }
}

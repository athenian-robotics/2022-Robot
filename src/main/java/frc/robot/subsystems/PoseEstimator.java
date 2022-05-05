package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.TimestampedPose2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class PoseEstimator extends SubsystemBase implements Loggable {
  private final DrivetrainSubsystem drivetrain;
  private final DifferentialDrivePoseEstimator poseEstimator;

  private TimestampedPose2d latestVisionPose;

  public PoseEstimator(DrivetrainSubsystem drivetrain, LimelightSubsystem limelight) {
    this.drivetrain = drivetrain;
    final Rotation2d initialHeading = drivetrain.getRotation2d();
    poseEstimator =
        new DifferentialDrivePoseEstimator(
            initialHeading,
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(1), 0.05, 0.05),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(1)),
            VecBuilder.fill(0.5, 0.5, Math.toRadians(30)));
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Log
  public Field2d getField() {
    Field2d field = new Field2d();
    field.setRobotPose(getPose());
    return field;
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(pose, drivetrain.getRotation2d());
  }

  public void addVisionPose(TimestampedPose2d pose) {
    this.latestVisionPose = pose;
    try {
      // poseEstimator.addVisionMeasurement(pose.pose, pose.timestamp);
    } catch (Exception ignored) {

    }
  }

  public Pose2d getVisionPose(boolean replaceIfStale) {
    if (replaceIfStale) {
      if (NetworkTablesJNI.now() - latestVisionPose.timestamp > 70) {
        return getPose();
      }
    }
    return latestVisionPose.pose;
  }

  @Override
  public void periodic() {
    DifferentialDriveWheelSpeeds wheelSpeeds = drivetrain.getWheelSpeeds();
    Rotation2d rotation = drivetrain.getRotation2d();
    double leftDistance = drivetrain.getLeftDistanceDriven();
    double rightDistance = drivetrain.getRightDistanceDriven();
    try {
      Pose2d localizationPose =
          poseEstimator.update(rotation, wheelSpeeds, leftDistance, rightDistance);

    } catch (Exception ignored) {
    }
  }
}

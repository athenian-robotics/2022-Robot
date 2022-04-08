package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase { // quick hack, should not extend subsytemBase
  private final DrivetrainSubsystem drivetrain;
  private final LimelightSubsystem limelight;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final DifferentialDriveOdometry odometry;

  public PoseEstimator(
      DrivetrainSubsystem drivetrain, LimelightSubsystem limelight, Pose2d initialPose) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    final Rotation2d initialHeading = drivetrain.getRotation2d();
    poseEstimator =
        new DifferentialDrivePoseEstimator(
            initialHeading,
            initialPose,
            VecBuilder.fill(0.05, 0.05, Math.toRadians(1), 0.05, 0.05),
            VecBuilder.fill(0.05, 0.05, Math.toRadians(1)),
            VecBuilder.fill(0.5, 0.5, Math.toRadians(10)));
    odometry = drivetrain.getOdometry();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // TODO: use vision to update pose estimator
  @Override
  public void periodic() {
    //    DifferentialDriveWheelSpeeds wheelSpeeds = drivetrain.getWheelSpeeds();
    //    Rotation2d rotation = drivetrain.getRotation2d();
    //    double leftDistance = drivetrain.getLeftDistanceDriven();
    //    double rightDistance = drivetrain.getRightDistanceDriven();
    //    try {
    //      Pose2d localizationPose =
    //          poseEstimator.update(rotation, wheelSpeeds, leftDistance, rightDistance);
    //    } catch (SingularMatrixException ignored) {
    //    }
  }
}

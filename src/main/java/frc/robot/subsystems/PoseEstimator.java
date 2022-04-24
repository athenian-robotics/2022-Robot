package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.TimestampedPose2d;
import org.ejml.data.SingularMatrixException;

public class PoseEstimator extends SubsystemBase {
  private final DrivetrainSubsystem drivetrain;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final LimelightSubsystem limelight;

  private final Field2d field = new Field2d();

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
    this.limelight = limelight;

    SmartDashboard.putData("field", field);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(pose, drivetrain.getRotation2d());
  }

  @Override
  public void periodic() {
    DifferentialDriveWheelSpeeds wheelSpeeds = drivetrain.getWheelSpeeds();
    Rotation2d rotation = drivetrain.getRotation2d();
    double leftDistance = drivetrain.getLeftDistanceDriven();
    double rightDistance = drivetrain.getRightDistanceDriven();
    TimestampedPose2d visionPose = limelight.getLatestPose();
    poseEstimator.addVisionMeasurement(visionPose.pose, visionPose.timestamp);
    try {
      Pose2d localizationPose =
          poseEstimator.update(rotation, wheelSpeeds, leftDistance, rightDistance);

    } catch (SingularMatrixException ignored) {
    }
    field.setRobotPose(getPose());
  }
}

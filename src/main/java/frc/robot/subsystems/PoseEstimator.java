package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.DifferentialDrivePoseEstimator;
import frc.robot.lib.TimestampedPose2d;
import frc.robot.lib.shooterData.ShooterDataTable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class PoseEstimator extends SubsystemBase implements Loggable {
  private final ShooterDataTable shooterDataTable;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final Encoder leftEncoder;
  private final Encoder rightEncoder;
  private final AHRS gyro;
  private Pose2d visionPose;

  private TimestampedPose2d latestVisionPose;

  public PoseEstimator(ShooterDataTable shooterDataTable) {
    this.shooterDataTable = shooterDataTable;

    leftEncoder =
        new Encoder(
            Constants.DriveConstants.leftEncoderChannelA,
            Constants.DriveConstants.leftEncoderChannelB,
            false);
    rightEncoder =
        new Encoder(
            Constants.DriveConstants.rightEncoderChannelA,
            Constants.DriveConstants.rightEncoderChannelB,
            true);
    gyro = new AHRS(SerialPort.Port.kMXP);

    leftEncoder.setDistancePerPulse(
        2 * 3.14 * (.1524 / 2) / 2048); // 6-inch wheel, to meters, PI for
    // circumference, gear conversion, 2048 ticks per rotation
    rightEncoder.setDistancePerPulse(
        2 * 3.14 * (.1524 / 2) / 2048); // 6-inch wheel, to meters, PI for
    // circumference, gear conversion, 2048 ticks per rotation

    final Rotation2d initialHeading = gyro.getRotation2d();
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
  @SuppressWarnings("PMD.UnusedPrivateMethod")
  public Field2d getField() {
    Field2d field = new Field2d();
    field.setRobotPose(getPose());
    return field;
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(pose, gyro.getRotation2d());
  }

  public void addVisionPose(Pose2d pose, double timestamp) {
    visionPose = pose;
    try {
      poseEstimator.addVisionMeasurement(pose, timestamp);
    } catch (Exception ignored) {

    }
  }

  //  public double getPositionOffset() {
  //    Pose2d pose = limelight.getLatestPose().pose;
  //    double velocity = kDriveKinematics.toChassisSpeeds(getWheelSpeeds()).vxMetersPerSecond;
  //    double distance = pose.getTranslation().getNorm();
  //    double TOF = shooterDataTable.getSpecs(Math.abs(distance)).getTOF();
  //    Translation2d displacementVector = new Pose2d().minus(pose).getTranslation();
  //    Translation2d linearVelocityVector = new Translation2d(velocity, new Rotation2d(0));
  //    double side = TOF * velocity;
  //    double tofOffset = Math.atan(side / distance);
  //    if (pose.getRotation().getDegrees() < 180) {
  //      tofOffset *= -1;
  //    }
  //    double angularVelocity =
  //        kDriveKinematics.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond;
  //    // cross product
  //    double cross =
  //        displacementVector.getX() * linearVelocityVector.getY()
  //            - displacementVector.getY() * linearVelocityVector.getX();
  //    return ((cross + angularVelocity) * Constants.looptime) - tofOffset;
  //  }

  //
  //  public static double getPositionOffset(
  //          double angle, double distance, double velocity, double TOF, double angularVelocity) {
  //    Translation2d displacementVector =
  //            new Translation2d(distance, new Rotation2d(Math.toRadians(-angle)));
  //    Translation2d linearVelocityVector = new Translation2d(velocity, new Rotation2d(0));
  //
  //    double side = TOF * velocity;
  //    double tofOffset = Math.atan(side / distance);
  //    if (angle < -180) {
  //      tofOffset *= -1;
  //    }
  //    // cross product
  //    double cross =
  //            displacementVector.getX() * linearVelocityVector.getY()
  //                    - displacementVector.getY() * linearVelocityVector.getX();
  //
  //    double v = ((cross + angularVelocity) * Constants.looptime) - tofOffset;
  //    SmartDashboard.putNumber("tof + turret offset", v);
  //    return v;
  //  }

  // public static void main(String[] args) {
  // System.out.println(Math.toDegrees(getPositionOffset(-90, 5, 1, 1.46, 0)));
  // }

  @Override
  public void periodic() {
    try {
      //      Pose2d localizationPose =
      //          poseEstimator.update(
      //              gyro.getRotation2d(),
      //              getWheelSpeeds(),
      //              leftEncoder.getDistance(),
      //              rightEncoder.getDistance());
    } catch (Exception ignored) {
    }
  }

  private DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public Pose2d getVisionPose() {
    return visionPose;
  }
}

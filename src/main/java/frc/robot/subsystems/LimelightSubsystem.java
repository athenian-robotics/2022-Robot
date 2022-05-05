package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.lib.TimestampedPose2d;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LimelightSubsystem extends SubsystemBase implements Loggable {
  @Log public double distance;
  @Log public double angleOffset;
  @Log public double timeSinceLastUpdate;
  final NetworkTable limelight; // used to be final?
  final TurretSubsystem turret;
  TimestampedPose2d pose = new TimestampedPose2d(new Pose2d(), 0);

  public LimelightSubsystem(String tableName, TurretSubsystem turret) {
    this.limelight = NetworkTableInstance.getDefault().getTable(tableName);

    limelight.addEntryListener(
        "llpython", this::updatePose, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    this.turret = turret;
    distance = 0;
  }

  private void updatePose(
      NetworkTable networkTable,
      String s,
      NetworkTableEntry networkTableEntry,
      NetworkTableValue networkTableValue,
      int i) {
    distance = networkTableValue.getDoubleArray()[0];
    angleOffset = networkTableValue.getDoubleArray()[1];
    long time = networkTableValue.getTime();
    timeSinceLastUpdate = Timer.getFPGATimestamp() - time;
    double angle = angleOffset + turret.getTurretAngleRadians();
    Pose2d pose =
        new Pose2d(
            new Translation2d(distance, new Rotation2d(angle - 90)),
            new Rotation2d(angle));
    this.pose = new TimestampedPose2d(pose, time);
    RobotContainer.poseEstimator.addVisionPose(this.pose);
  }

  @Log
  public Field2d getField() {
    Field2d field = new Field2d();
    field.setRobotPose(pose.pose);
    return field;
  }

  public TimestampedPose2d getLatestPose() {
    return this.pose;
  }

  // i hate limelightdatalatch
}

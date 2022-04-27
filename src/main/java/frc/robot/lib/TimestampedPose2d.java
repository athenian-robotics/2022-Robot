package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;

/** A timestamped pose. */
public class TimestampedPose2d {
  public final Pose2d pose;
  public final double timestamp;

  public TimestampedPose2d(Pose2d pose, double timestamp) {
    this.pose = pose;
    this.timestamp = timestamp;
  }
}

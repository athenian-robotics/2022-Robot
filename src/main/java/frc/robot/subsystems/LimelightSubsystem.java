package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.TimestampedPose2d;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import java.util.LinkedList;

// YOU DON'T NEED TO REQUIRE LIMELIGHTSUBSYSTEM IN COMMANDS!
public class LimelightSubsystem extends SubsystemBase {
  public final LimelightDataLatchManager latchManager = new LimelightDataLatchManager();
  final NetworkTable limelight; // used to be final?
  final TurretSubsystem turret;
  TimestampedPose2d pose = new TimestampedPose2d(new Pose2d(), 0);

  public LimelightSubsystem(String tableName, TurretSubsystem turret) {
    this.limelight = NetworkTableInstance.getDefault().getTable(tableName);

    limelight.addEntryListener(
        "llpython", this::updatePose, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    this.turret = turret;
  }

  private void updatePose(
      NetworkTable networkTable,
      String s,
      NetworkTableEntry networkTableEntry,
      NetworkTableValue networkTableValue,
      int i) {
    double distance = networkTableValue.getDoubleArray()[0];
    double angleOffset = networkTableValue.getDoubleArray()[1];
    long time = networkTableValue.getTime();
    double angle = angleOffset + turret.getTurretAngleRadians();
    Pose2d pose =
        new Pose2d(
            new Translation2d(distance, new Rotation2d(Math.PI / 2 + angle)),
            new Rotation2d(-angle));
    this.pose = new TimestampedPose2d(pose, time);
  }

  public TimestampedPose2d getLatestPose() {
    return this.pose;
  }

  public void addLatch(LimelightDataLatch latch) {
    latchManager.addLatch(latch);
  }

  public void disable() {
    latchManager.clearPool();
  }

  // i hate limelightdatalatch

  public void periodic() {
    Double[] lloutput =
        (Double[])
            limelight
                .getEntry("llpython")
                .getNumberArray(new Number[] {-1, -1, -1, -1, -1, -1, -1, -9});
    latchManager.update(lloutput);
  }

  // Keeps a record of Latches waiting to be opened, and opens them when valid data comes along (see
  // periodic())
  private static class LimelightDataLatchManager {
    private final LinkedList<LimelightDataLatch> latchPool = new LinkedList<>();

    // Updates every pooled latch if there's new data
    public void update(Double[] limelightOutputArray) {
      if (limelightOutputArray.length == 8 && limelightOutputArray[7] == (double) 1) {
        while (latchPool.size() != 0) {
          LimelightDataLatch currentLatch = latchPool.pollFirst();
          currentLatch.unlock(limelightOutputArray[currentLatch.limelightDataType.llpythonIndex]);
          if (currentLatch.limelightDataType == LimelightDataType.DISTANCE)
            SmartDashboard.putNumber(
                "Limelight Distance",
                limelightOutputArray[currentLatch.limelightDataType.llpythonIndex]);
          else
            SmartDashboard.putNumber(
                "Limelight Offset Degrees",
                Math.toDegrees(limelightOutputArray[currentLatch.limelightDataType.llpythonIndex]));
        }
      } else {
        latchPool.removeIf(LimelightDataLatch::expired);
      }
    }

    private void addLatch(LimelightDataLatch latch) {
      latchPool.addFirst(latch);
    }

    private void clearPool() {
      latchPool.clear();
    }
  }
}

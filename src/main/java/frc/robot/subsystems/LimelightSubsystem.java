package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import java.util.LinkedList;

// YOU DON'T NEED TO REQUIRE LIMELIGHTSUBSYSTEM IN COMMANDS!
public class LimelightSubsystem extends SubsystemBase {
  public final LimelightDataLatchManager latchManager = new LimelightDataLatchManager();
  final NetworkTable limelight; // used to be final?

  public LimelightSubsystem(String tableName) {
    this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
  }

  public void addLatch(LimelightDataLatch latch) {
    latchManager.addLatch(latch);
  }

  public void disable() {
    latchManager.clearPool();
  }
  //  TODO: get absolute position and tweak pose estimator
  //  public Pose2d getLimelightPose() {
  //    return new Pose2d(
  //
  //    );
  //  }

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

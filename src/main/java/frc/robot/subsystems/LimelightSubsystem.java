package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.limelight.LimelightDataLatch;

import java.util.ArrayList;
import java.util.LinkedList;


//YOU DON'T NEED TO REQUIRE LIMELIGHTSUBSYSTEM IN COMMANDS!
public class LimelightSubsystem extends SubsystemBase {
    public final LimelightDataLatchManager latchManager = new LimelightDataLatchManager();
    final NetworkTable limelight; //used to be final?

    public LimelightSubsystem(String tableName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public void addLatch(LimelightDataLatch latch) {
        latchManager.addLatch(latch);
    }

    public boolean checkGoalNotFound() {
        Double[] lloutput = (Double[]) limelight.getEntry("llpython").getNumberArray(new Number[] {-1, -1, -1, -1, -1
                , -1, -1, -9});
        return lloutput[7] == (double) -1;
    }


    public void disable() {latchManager.clearPool();}

    public void periodic() {
        Double[] lloutput = (Double[]) limelight.getEntry("llpython").getNumberArray(new Number[] {-1, -1, -1, -1, -1
                , -1, -1, -9});
        latchManager.update(lloutput);
        SmartDashboard.putNumber("X Offset", lloutput[1]);
        SmartDashboard.putNumber("Distance from Target", lloutput[0]);
    }


    //Keeps a record of Latches waiting to be opened, and opens them when valid data comes along (see periodic())
    private static class LimelightDataLatchManager {
        private final ArrayList<LimelightDataLatch> latchPool = new ArrayList<>();

        //Updates every pooled latch if there's new data
        public void update(Double[] limelightOutputArray) {
            if (limelightOutputArray.length == 8 && limelightOutputArray[7] == (double) 1) {
                while (latchPool.size() != 0) {
                    LimelightDataLatch currentLatch = latchPool.remove(0);
                    currentLatch.unlock(limelightOutputArray[currentLatch.limelightDataType.llpythonIndex]);
                }
            } else {
                latchPool.removeIf(LimelightDataLatch::expired);
            }
        }

        private void addLatch(LimelightDataLatch latch) {
            latchPool.add(latch);
        }

        private void clearPool() {
            latchPool.clear();
        }
    }
}

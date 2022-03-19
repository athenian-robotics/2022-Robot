package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.limelight.LimelightDataLatch;

import java.util.Iterator;
import java.util.LinkedList;


//YOU DON'T NEED TO REQUIRE LIMELIGHTSUBSYSTEM IN COMMANDS!
public class LimelightSubsystem extends SubsystemBase {
    public final LimelightDataLatchManager latchManager = new LimelightDataLatchManager();
    NetworkTable limelight; //used to be final?
    final NetworkTableEntry xOffsetNTE;
    final NetworkTableEntry distanceNTE;

    public LimelightSubsystem(String tableName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
        xOffsetNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("X Offset", 0)
                .getEntry();
        distanceNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Distance to Target", 0)
                .getEntry();
    }

    public void addLatch(LimelightDataLatch latch) {
        latchManager.addLatch(latch);
    }

    public boolean checkGoalNotFound()
        {
        Double[] lloutput = (Double[]) limelight.getEntry("llpython").getNumberArray(new Number[]{-1, -1, -1, -1, -1, -1, -1, -9});
        return lloutput[7] == (double) -1;
        };


    public void disable() {latchManager.clearPool();}

    public void periodic() {
        Double[] lloutput = (Double[]) limelight.getEntry("llpython").getNumberArray(new Number[]{-1, -1, -1, -1, -1, -1, -1, -9});
        latchManager.update(lloutput);
        xOffsetNTE.setDouble(lloutput[1]);
        distanceNTE.setDouble(lloutput[0]);
        SmartDashboard.putNumber("xOffset", lloutput[1]);
        SmartDashboard.putNumber("zDistance", lloutput[0]);
    }


    //Keeps a record of Latches waiting to be opened, and opens them when valid data comes along (see periodic())
    private static class LimelightDataLatchManager {
        private final LinkedList<LimelightDataLatch> latchPool = new LinkedList<>();

        //Updates every pooled latch if there's new data
        public void update(Double[] limelightOutputArray) {
            if (limelightOutputArray.length == 8 && limelightOutputArray[7] == (double) 1) {
                while (latchPool.size() != 0) {
                    LimelightDataLatch currentLatch = latchPool.pollFirst();
                    currentLatch.unlock(limelightOutputArray[currentLatch.limelightDataType.llpythonIndex]);
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

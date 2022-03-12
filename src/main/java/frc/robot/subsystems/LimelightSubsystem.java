package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.limelight.GoalNotFoundException;


public class LimelightSubsystem extends SubsystemBase {
    final NetworkTable limelight;
    private Number[] limelightOutputArray = {-1, -1, -1, -1, -1, -1, -1, -1};

    public LimelightSubsystem(String tableName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public double getLimelightOutputAtIndex(int index) throws GoalNotFoundException {
        if (index > 8 || index < 0) {
            throw new IndexOutOfBoundsException();
        } else if (Math.abs(limelightOutputArray[7].doubleValue() - 1) < 0.1) {
            return (double) limelightOutputArray[index];
        } else {
            throw new GoalNotFoundException();
        }
    }

     public double getDistance() throws GoalNotFoundException {
        if (isTargetFound()) return getLimelightOutputAtIndex(0);
        else throw new GoalNotFoundException();
    }

    public boolean isTargetFound() {
        try {
            return (double) getLimelightOutputAtIndex(7) == (double) 1;
        } catch (GoalNotFoundException e) {
            return false;
        }
    }

    public void disable() {}

    public void periodic() {
        try {
            Number[] temp = limelight.getEntry("llpython").getNumberArray(new Number[]{-1, -1, -1, -1, -1, -1, -1, -9});
            if (temp.length == 8 && (double) temp[7] != (double) -1) {
                limelightOutputArray = temp;
            }
        } catch (ArrayIndexOutOfBoundsException ignored) {}
        try {
            SmartDashboard.putNumber("xOffset", getLimelightOutputAtIndex(1));
            SmartDashboard.putNumber("zDistance", getLimelightOutputAtIndex(0));
        } catch (GoalNotFoundException ignored) {}
    }
}

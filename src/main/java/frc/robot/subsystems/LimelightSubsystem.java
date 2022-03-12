package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.GoalNotFoundException;

import java.util.Arrays;
import java.util.Objects;


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

     public double getDistance() {
        try {
            return getLimelightOutputAtIndex(0);
        } catch (GoalNotFoundException e) {
            return -3;
        }
    }

    public boolean isTargetFound() {
        return (double) limelightOutputArray[7] == (double) 1;
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

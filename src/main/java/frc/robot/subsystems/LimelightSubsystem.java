package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.GoalNotFoundException;

import java.util.Objects;

public class LimelightSubsystem extends SubsystemBase {
    final NetworkTable limelight;
    Number[] limelightOutputArray = {-1, -1, -1, -1, -1, -1, -1, -1};

    public LimelightSubsystem(String tableName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public double getLimelightOutputAtIndex(int index) throws GoalNotFoundException {
        if (index > 8 || index < 0) {
            throw new IndexOutOfBoundsException();
        } else if ((int) limelightOutputArray[7] == 1) {
            return (double) limelightOutputArray[index];
        } else {
            throw new GoalNotFoundException();
        }
    }

    public void disable() {}

    public void periodic() {
        try {
            Number[] temp = limelight.getEntry("llpython").getNumberArray(new Number[]{-1, -1, -1, -1, -1, -1, -1, -1});
            if (Objects.equals(temp[7], 1) || Objects.equals(temp[7], 0)) limelightOutputArray = temp;
        } catch (ArrayIndexOutOfBoundsException ignored) {}
        try {
            SmartDashboard.putNumber("xOffset", getLimelightOutputAtIndex(1));
        } catch (GoalNotFoundException ignored) {}
    }
    public double getDistance() {
        try {
            return getLimelightOutputAtIndex(0); // TODO: check if index is correct
        } catch (GoalNotFoundException e) {
            return -1;
        }
    }
    public boolean isTargetFound() {

        try {
            return getLimelightOutputAtIndex(7) == 1;
        } catch (GoalNotFoundException e) {
            return false;
        }
    }
}

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.GoalNotFoundException;

public class LimelightSubsystem extends SubsystemBase {
    final NetworkTable limelight;
    Number[] limelightOutputArray;
    Number[] defaultLimelightOutputArray = {-1, -1, -1, -1, -1, -1, -1, -1};

    public LimelightSubsystem(String tableName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
        limelightOutputArray = defaultLimelightOutputArray;
    }

    public double getLimelightOutputAtIndex(int index) throws GoalNotFoundException {
        if (index > 8 || index < 0) {
            throw new IndexOutOfBoundsException();
        } else if (limelightOutputArray != defaultLimelightOutputArray) {
            return (double) limelightOutputArray[index];
        } else {
            throw new GoalNotFoundException();
        }
    }

    public void disable() {
    }

    public void periodic() {
        limelightOutputArray = limelight.getEntry("llpython").getNumberArray(defaultLimelightOutputArray);
        try {
            SmartDashboard.putNumber("xOffset", getLimelightOutputAtIndex(1));
        } catch (GoalNotFoundException ignored) {
        }
    }
}

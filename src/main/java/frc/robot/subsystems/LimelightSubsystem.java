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
            //System.out.println(limelightOutputArray[7]);
            throw new GoalNotFoundException();
        }
    }

    public void disable() {}

    public void periodic() {
        try {
            Number[] temp = limelight.getEntry("llpython").getNumberArray(new Number[]{-1, -1, -1, -1, -1, -1, -1, -1});
            if (Objects.equals(temp[7], (Number) 1)) limelightOutputArray = temp;
        } catch (ArrayIndexOutOfBoundsException e) {
        }

        try {
            SmartDashboard.putNumber("xOffset", getLimelightOutputAtIndex(1));
        } catch (GoalNotFoundException ignored) {
            //  System.out.println("xOffset not found!");
        }
    }
}

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
        try{
            System.out.println("\n\n\n\n\n" + this.limelight + "\n\n" + this.limelight.getEntry("llpython").getNumberArray(new Number[]{-1. -1, -1, -1, -1, -1, -1, -1})[1]);
        }catch(Exception e){
            System.out.println("Exception found");
        }
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
            if (!Objects.equals(temp[7], (new Double(-1.0))) || Objects.equals(temp[7], 0)) limelightOutputArray = temp;
        } catch (ArrayIndexOutOfBoundsException ignored) {}
        try {
            SmartDashboard.putNumber("xOffset", getLimelightOutputAtIndex(1));
            SmartDashboard.putNumber("zDistance", getLimelightOutputAtIndex(0));
        } catch (GoalNotFoundException ignored) {}
    }
}

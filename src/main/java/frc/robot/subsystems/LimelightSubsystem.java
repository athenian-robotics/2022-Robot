package frc.robot.subsystems;

import edu.wpi.first.networktables.*;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.auto.components.GoalNotFoundException;

public class LimelightSubsystem extends SubsystemBase {
    final NetworkTable limelight;
    double[] limelightOutputArray;
    double[] defaultLimelightOutputArray = {-1, -1, -1, -1, -1, -1, -1, -1};

    public LimelightSubsystem(String tableName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
        limelightOutputArray = defaultLimelightOutputArray;
    }

    public double getLimelightOutputAtIndex(int index) throws GoalNotFoundException {
        if (index>8||index<0) {
            throw new IndexOutOfBoundsException();
        } else if (limelightOutputArray!=defaultLimelightOutputArray) {
            return limelightOutputArray[index];
        } else {
            throw new GoalNotFoundException();
        }
    }

    public void periodic() {
        limelightOutputArray = limelight.getEntry("llpython").getDoubleArray(defaultLimelightOutputArray);
        System.out.println(limelightOutputArray[1]);
        SmartDashboard.putNumber("xOffset", limelightOutputArray[1]);
    }

    public void disable() {
    }
}
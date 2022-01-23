package frc.robot.subsystems;

import edu.wpi.first.networktables.*;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    final NetworkTable limelight;

    public LimelightSubsystem(String tableName) {
        this.limelight = NetworkTableInstance.getDefault().getTable(tableName);
        this.limelight.addEntryListener(new TableEntryListener() {
            @Override
            public void valueChanged(NetworkTable table, String key, NetworkTableEntry entry, NetworkTableValue value, int flags) {
            }
        }, EntryListenerFlags.kUpdate);
    }

    public void periodic() {

    }

}


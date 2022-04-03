package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


//If limelight data isn't found within a quarter-second, the default hood angle will be set.
public class SetHoodAngleWithLimelight extends CommandBase {
    private final ShooterDataTable shooterDataTable;
    private final LimelightSubsystem limelightSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final LimelightDataLatch distanceLatch;

    public SetHoodAngleWithLimelight(ShooterDataTable shooterDataTable, LimelightSubsystem limelightSubsystem, HoodSubsystem hoodSubsystem) {
        this.shooterDataTable = shooterDataTable;
        this.limelightSubsystem = limelightSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.distanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE);
        addRequirements(this.hoodSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.addLatch(distanceLatch);
    }

    @Override
    public boolean isFinished() {
        try {
            return distanceLatch.unlocked();
        } catch (GoalNotFoundException e) {
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.setHoodAngle(shooterDataTable.getSpecs(distanceLatch.open()).getAngle());
    }
}

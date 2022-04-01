package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


//If limelight data isn't found within a quarter-second, the default hood angle will be set.
public class SetHoodAngleWithLimelight extends CommandBase {
    private final ShooterDataTable shooterDataTable;
    private final LimelightSubsystem limelightSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final LimelightDataLatch latch;

    public SetHoodAngleWithLimelight(ShooterDataTable shooterDataTable, LimelightSubsystem limelightSubsystem,
                                     ShooterSubsystem shooterSubsystem) {
        this.shooterDataTable = shooterDataTable;
        this.limelightSubsystem = limelightSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.latch = new LimelightDataLatch(LimelightDataType.DISTANCE);
        addRequirements();
    }

    @Override
    public void initialize() {
        limelightSubsystem.addLatch(latch);
    }

    @Override
    public boolean isFinished() {
        try {
            return latch.unlocked();
        } catch (GoalNotFoundException e) {
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setHoodAngle(shooterDataTable.getSpecs(latch.open()).getAngle());
    }
}

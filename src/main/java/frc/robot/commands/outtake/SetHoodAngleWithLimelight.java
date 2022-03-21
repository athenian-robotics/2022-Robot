package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;


//If limelight data isn't found within a quarter-second, the default hood angle will be set.
public class SetHoodAngleWithLimelight extends CommandBase {
    private final ShooterDataTable shooterDataTable;
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final LimelightDataLatch latch;
    private double angle;
    private long timeout = Long.MAX_VALUE;

    public SetHoodAngleWithLimelight(ShooterDataTable shooterDataTable, LimelightSubsystem limelightSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.shooterDataTable = shooterDataTable;
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.latch = new LimelightDataLatch(LimelightDataType.DISTANCE);
        addRequirements(this.outtakeSubsystem);
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
        outtakeSubsystem.setHoodAngle(shooterDataTable.getSpecs(latch.open()).getAngle());
    }
}

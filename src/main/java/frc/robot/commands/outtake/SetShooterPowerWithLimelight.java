package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;


//If limelight data isn't found within a quarter-second, the default shooter power will be set.
public class SetShooterPowerWithLimelight extends CommandBase {
    private final ShooterDataTable shooterDataTable;
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final LimelightDataLatch latch;

    public SetShooterPowerWithLimelight(ShooterDataTable shooterDataTable, LimelightSubsystem limelightSubsystem,
                                        OuttakeSubsystem outtakeSubsystem) {
        this.shooterDataTable = shooterDataTable;
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.latch = new LimelightDataLatch(LimelightDataType.DISTANCE);
        addRequirements();
    }

    @Override
    public void initialize() {
        limelightSubsystem.addLatch(latch);
    }

    @Override
    public boolean isFinished() {
        try { //jump to end() as soon as we get data! If we receive none in time, we'll use the default given by
            // shooterDataTable.getSpecs(0.0d).getPower()
            return latch.unlocked();
        } catch (GoalNotFoundException e) {return true;}
    }

    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.setRPS(shooterDataTable.getSpecs(latch.open()).getPower());
    }
}

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;


public class GuaranteeLimelightDataEquals extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final LimelightDataLatch dataLatch;
    private final double valueToGuarantee;
    private final double tolerance;

    public GuaranteeLimelightDataEquals(LimelightSubsystem limelightSubsystem, LimelightDataType dataType, double valueToGuarantee, double tolerance) {
        this.limelightSubsystem = limelightSubsystem;
        this.valueToGuarantee = valueToGuarantee;
        this.tolerance = tolerance;
        this.dataLatch = new LimelightDataLatch(dataType);
    }

    @Override
    public void initialize() {
        limelightSubsystem.addLatch(dataLatch);
    }

    @Override
    public boolean isFinished() {
        try {
            if (dataLatch.unlocked()) {
                if (Math.abs(dataLatch.open() - valueToGuarantee) < tolerance) {
                    return true;
                } else {
                    throw new GoalNotFoundException(); //shortcut to reset and reschedule latch vvv
                }
            } else {
                return false;
            }
        } catch (GoalNotFoundException e) {
            limelightSubsystem.addLatch(dataLatch.reset());
            return false;
        }
    }
}

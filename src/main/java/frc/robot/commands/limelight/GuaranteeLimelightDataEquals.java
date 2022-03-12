package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.subsystems.LimelightSubsystem;


public class GuaranteeLimelightDataEquals extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final double valueToGuarantee;
    private final double tolerance;
    private final int index;

    public GuaranteeLimelightDataEquals(LimelightSubsystem limelightSubsystem, int index, double valueToGuarantee) {
        this.limelightSubsystem = limelightSubsystem; this.index = index; this.valueToGuarantee = valueToGuarantee; this.tolerance = 1;
        addRequirements(this.limelightSubsystem);
    }

    public GuaranteeLimelightDataEquals(LimelightSubsystem limelightSubsystem, int index, double valueToGuarantee, double tolerance) {
        this.limelightSubsystem = limelightSubsystem; this.index = index; this.valueToGuarantee = valueToGuarantee; this.tolerance = tolerance;
        addRequirements(this.limelightSubsystem);
    }

    @Override
    public boolean isFinished() {
        try {
            return Math.abs(limelightSubsystem.getLimelightOutputAtIndex(index) - valueToGuarantee) < tolerance;
        } catch (GoalNotFoundException e) {
            return false;
        }
    }
}

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.subsystems.LimelightSubsystem;


public class GuaranteeLimelightData extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;

    public GuaranteeLimelightData(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(this.limelightSubsystem);
    }

    @Override
    public boolean isFinished() {
        try {
            return (double) limelightSubsystem.getLimelightOutputAtIndex(7) == (double) 1;
        } catch (GoalNotFoundException e) {
            return false;
        }
    }
}

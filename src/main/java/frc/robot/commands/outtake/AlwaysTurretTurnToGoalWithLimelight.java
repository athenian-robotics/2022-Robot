package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;


public class AlwaysTurretTurnToGoalWithLimelight extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final LimelightDataLatch offsetLatch;

    public AlwaysTurretTurnToGoalWithLimelight(LimelightSubsystem limelightSubsystem,
                                               OuttakeSubsystem outtakeSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.addLatch(offsetLatch.reset());
    }

    @Override
    public void execute() {
        try {
            if (offsetLatch.unlocked()) {
                outtakeSubsystem.setTurretPositionRadians(offsetLatch.open() + outtakeSubsystem.getTurretAngleRadians());
                throw new GoalNotFoundException(); //shortcut to latch reset  vvv  (since we've expended it)
            }
        } catch (GoalNotFoundException e) {
            limelightSubsystem.addLatch(offsetLatch.reset()); //assuming we want to look for the goal forever
        }
    }
}

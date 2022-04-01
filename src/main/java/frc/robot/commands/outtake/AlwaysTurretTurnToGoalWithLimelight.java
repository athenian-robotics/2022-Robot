package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class AlwaysTurretTurnToGoalWithLimelight extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final LimelightDataLatch offsetLatch;

    public AlwaysTurretTurnToGoalWithLimelight(LimelightSubsystem limelightSubsystem,
                                               ShooterSubsystem shooterSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.addLatch(offsetLatch.reset());
    }

    @Override
    public void execute() {
        try {
            if (offsetLatch.unlocked()) {
                shooterSubsystem.setTurretPositionRadians(offsetLatch.open() + shooterSubsystem.getTurretAngleRadians());
                throw new GoalNotFoundException(); //shortcut to latch reset  vvv  (since we've expended it)
            }
        } catch (GoalNotFoundException e) {
            limelightSubsystem.addLatch(offsetLatch.reset()); //assuming we want to look for the goal forever
        }
    }
}

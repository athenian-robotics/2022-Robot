package frc.robot.commands.outtake;

import frc.robot.Constants;
import frc.robot.lib.DetourableCommand;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.*;


public class TurretTurnToGoalWithLimelight extends DetourableCommand {
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final LimelightDataLatch distanceLatch, offsetLatch;

    public TurretTurnToGoalWithLimelight(LimelightSubsystem limelightSubsystem, OuttakeSubsystem outtakeSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
        distanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 5);
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void _initialize() {
        limelightSubsystem.addLatch(offsetLatch);
        limelightSubsystem.addLatch(distanceLatch);
    }

    @Override
    public void _execute() {}

    @Override
    public boolean _isFinished() {
        try {
            if (offsetLatch.unlocked() && distanceLatch.unlocked()) {
                double offset = offsetLatch.open();
                if (Math.abs(offset) < 4/distanceLatch.open()) return true;
                if (offset > 0 && outtakeSubsystem.getTurretAngle() > maximumTurretAngle) detour(new TurretTurnToAngle(outtakeSubsystem, minimumHoodAngle + 20));
                else if (offset < 0 && outtakeSubsystem.getTurretAngle() < minimumHoodAngle) detour(new TurretTurnToAngle(outtakeSubsystem, maximumHoodAngle - 20));
                else detour(new TurretTurnToAngle(outtakeSubsystem, offset));
                throw new GoalNotFoundException(); //shortcut to latch reset  vvv  (since we've expended them)
            } else {
                return false;
            }
        } catch (GoalNotFoundException e) {
            if (offsetLatch.expired()) limelightSubsystem.addLatch(offsetLatch.reset()); //assuming we want to look for the goal forever
            if (distanceLatch.expired()) limelightSubsystem.addLatch(distanceLatch.reset());
            return false;
        }
    }

    @Override
    public void _end(boolean interrupted) {}
}

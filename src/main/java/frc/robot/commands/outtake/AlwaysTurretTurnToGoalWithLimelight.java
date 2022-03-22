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
    private LimelightDataLatch offsetLatch;
    private double offset = Double.MAX_VALUE;

    public AlwaysTurretTurnToGoalWithLimelight(LimelightSubsystem limelightSubsystem,
                                               OuttakeSubsystem outtakeSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        outtakeSubsystem.limelightTurretAnglePID.reset();
        limelightSubsystem.addLatch(offsetLatch.reset());
        offset = Double.MAX_VALUE;
    }

    @Override
    public void execute() {
        try {
            if (offsetLatch.unlocked()) {
                offset = offsetLatch.open();
                outtakeSubsystem.turnTurret(-outtakeSubsystem.limelightTurretAnglePID.calculate(offset));
                throw new GoalNotFoundException(); //shortcut to latch reset  vvv  (since we've expended it)
            }
        } catch (GoalNotFoundException e) {
            offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
            limelightSubsystem.addLatch(offsetLatch); //assuming we want to look for the goal forever
        }
    }
}

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.GoalNotFoundException;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;


public class TurretTurnToGoal extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private double distance = 256;

    public TurretTurnToGoal(LimelightSubsystem limelightSubsystem, OuttakeSubsystem outtakeSubsystem) throws GoalNotFoundException {
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.limelightSubsystem, this.outtakeSubsystem);
    }

    @Override
    public void execute() {
        if (limelightSubsystem.isTargetFound()) {
            try {
                outtakeSubsystem.turnTurret(outtakeSubsystem.turretAnglePID.calculate(limelightSubsystem.getLimelightOutputAtIndex(1)));
            } catch (GoalNotFoundException ignored) {}
        } else {
            this.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        try {
            return Math.abs(limelightSubsystem.getLimelightOutputAtIndex(1)) < 5/distance;
        } catch (GoalNotFoundException e) {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.turnTurret(0);
    }
}

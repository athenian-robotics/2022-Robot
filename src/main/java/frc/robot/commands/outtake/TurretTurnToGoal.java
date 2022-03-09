package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyScheduleCommand;
import frc.robot.lib.GoalNotFoundException;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.maximumTurretAngle;
import static frc.robot.Constants.MechanismConstants.minimumTurretAngle;


public class TurretTurnToGoal extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final double hardstopDeadzoneBuffer = 10;
    private final double hardstopMidpoint = (maximumTurretAngle + minimumTurretAngle) / 2;
    private long hardstopToggleCountdown = Long.MAX_VALUE;
    private double distance = 256;

    public TurretTurnToGoal(LimelightSubsystem limelightSubsystem, OuttakeSubsystem outtakeSubsystem) throws GoalNotFoundException {
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.limelightSubsystem, this.outtakeSubsystem);
    }

    @Override
    public void execute() {
        if (System.currentTimeMillis() - hardstopToggleCountdown > 1000) {
            new TurretTurnToAngle(outtakeSubsystem, outtakeSubsystem.getTurretPosition() > hardstopMidpoint ? minimumTurretAngle + 20 : maximumTurretAngle - 20).schedule();
        } else if (limelightSubsystem.isTargetFound()) {
            try {
                double goalOffset = limelightSubsystem.getLimelightOutputAtIndex(1);
                outtakeSubsystem.turnTurret(-outtakeSubsystem.turretAnglePID.calculate(goalOffset));

                if (outtakeSubsystem.getTurretPosition() > maximumTurretAngle - hardstopDeadzoneBuffer && goalOffset > hardstopDeadzoneBuffer
                        || outtakeSubsystem.getTurretPosition() < minimumTurretAngle + hardstopDeadzoneBuffer && goalOffset < hardstopDeadzoneBuffer) {
                    hardstopToggleCountdown = System.currentTimeMillis();
                } else {
                    hardstopToggleCountdown = Long.MAX_VALUE;
                }
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

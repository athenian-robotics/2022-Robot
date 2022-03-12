package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.minimumTurretAngle;
import static frc.robot.Constants.MechanismConstants.maximumTurretAngle;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;


public class TurretTurnToGoalOrManualControl extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private static final double hardstopMidpoint = (maximumTurretAngle + minimumTurretAngle) / 2;
    private double inactivityTime = Long.MAX_VALUE;

    public TurretTurnToGoalOrManualControl(OuttakeSubsystem outtakeSubsystem, LimelightSubsystem limelightSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(this.outtakeSubsystem, this.limelightSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (FightStick.fightStickJoystick.getX() < -0.75) {
            outtakeSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.75) {
            outtakeSubsystem.turnTurret(turretTurnSpeed);
        } else if (System.currentTimeMillis() - inactivityTime > 200) {
            outtakeSubsystem.turnTurret(0);
        } else if (limelightSubsystem.isTargetFound()) {
            try {
                double goalOffset = limelightSubsystem.getLimelightOutputAtIndex(1);
                outtakeSubsystem.turnTurret(-outtakeSubsystem.turretAnglePID.calculate(goalOffset));
                if (outtakeSubsystem.getTurretPosition() > maximumTurretAngle && goalOffset > 0) new TurretTurnToAngle(outtakeSubsystem, minimumTurretAngle + 20);
                if (outtakeSubsystem.getTurretPosition() < minimumTurretAngle && goalOffset < 0) new TurretTurnToAngle(outtakeSubsystem, maximumTurretAngle - 20);
            } catch (GoalNotFoundException ignored) {}
        } else if (inactivityTime == Long.MAX_VALUE) {
            inactivityTime = System.currentTimeMillis();
        } else {

        }
    }

    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.turnTurret(0);
    }
}

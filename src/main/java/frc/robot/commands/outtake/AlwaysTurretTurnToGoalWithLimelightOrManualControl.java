package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.slowTurretTurnSpeed;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;


public class AlwaysTurretTurnToGoalWithLimelightOrManualControl extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final LimelightDataLatch offsetLatch;

    public AlwaysTurretTurnToGoalWithLimelightOrManualControl(LimelightSubsystem limelightSubsystem, OuttakeSubsystem outtakeSubsystem) {
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
        SmartDashboard.putBoolean("flight stick share", !FightStick.fightStickShare.getAsBoolean());
        if (FightStick.fightStickJoystick.getX() < -0.5) { //TURRET ADJUSTMENT FALCON
            outtakeSubsystem.turretRunning = false;
            outtakeSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.5) {
            outtakeSubsystem.turretRunning = false;
            outtakeSubsystem.turnTurret(turretTurnSpeed);
        }else if (FightStick.fightStickJoystick.getY() < -0.5) { //TURRET ADJUSTMENT FALCON
                outtakeSubsystem.turretRunning = false;
                outtakeSubsystem.turnTurret(-slowTurretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() > 0.5) {
                outtakeSubsystem.turretRunning = false;
                outtakeSubsystem.turnTurret(slowTurretTurnSpeed);
        } else if (FightStick.fightStickShare.getAsBoolean()){
            try {
                if (offsetLatch.unlocked()) {
                    outtakeSubsystem.turnTurret(
                            Math.abs(offsetLatch.open()-outtakeSubsystem.getTurretAngle()) > Math.PI/90
                                    ? Math.signum(offsetLatch.open()) * turretTurnSpeed
                                    : Math.signum(offsetLatch.open()) * slowTurretTurnSpeed);
                    throw new GoalNotFoundException(); //shortcut to latch reset  vvv  (since we've expended it)
                }
            } catch (GoalNotFoundException e) {
                limelightSubsystem.addLatch(offsetLatch.reset()); //assuming we want to look for the goal forever
            }
        } else {
            outtakeSubsystem.stopTurret();
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.stopTurret();
    }
}

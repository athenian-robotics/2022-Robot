package frc.robot.commands.outtake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.*;
import static frc.robot.Constants.looptime;


public class AlwaysTurretTurnToGoalWithLimelightAndSetHoodAngleOrManualControl extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final ShooterDataTable shooterDataTable;
    private final LimelightDataLatch offsetLatch;
    private final LimelightDataLatch distanceLatch;
    private final LinearFilter filter;
    private double offset;

    public AlwaysTurretTurnToGoalWithLimelightAndSetHoodAngleOrManualControl(LimelightSubsystem limelightSubsystem,
                                                                             OuttakeSubsystem outtakeSubsystem,
                                                                             ShooterDataTable shooterDataTable) {
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.shooterDataTable = shooterDataTable;
        offsetLatch = new LimelightDataLatch(LimelightDataType.HORIZONTAL_OFFSET, 5);
        distanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 5);
        addRequirements(this.outtakeSubsystem);
        filter = LinearFilter.singlePoleIIR(0.1, looptime); // time constant is lag, tune later
    }

    @Override
    public void initialize() {
        limelightSubsystem.addLatch(offsetLatch.reset());
        limelightSubsystem.addLatch(distanceLatch.reset());
    }

    @Override
    public void execute() {
        try {
            if (offsetLatch.unlocked()) {
                offset = offsetLatch.open();
            }
        } catch (GoalNotFoundException e) {
           limelightSubsystem.addLatch(offsetLatch.reset());
        }
        if (FightStick.fightStickJoystick.getX() < -0.5) { //TURRET ADJUSTMENT FALCON
            outtakeSubsystem.turretRunning = false;
            outtakeSubsystem.lqrRunning = false;
            outtakeSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.5) {
            outtakeSubsystem.turretRunning = false;
            outtakeSubsystem.lqrRunning = false;
            outtakeSubsystem.turnTurret(turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() < -0.5) { //TURRET ADJUSTMENT FALCON
            outtakeSubsystem.turretRunning = false;
            outtakeSubsystem.lqrRunning = false;
            outtakeSubsystem.turnTurret(-slowTurretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() > 0.5) {
            outtakeSubsystem.turretRunning = false;
            outtakeSubsystem.lqrRunning = false;
            outtakeSubsystem.turnTurret(slowTurretTurnSpeed);
        } else if (FightStick.fightStickShare.get()) {
                    outtakeSubsystem.setTurretPositionRadians(filter.calculate(offset) + outtakeSubsystem.getTurretAngleRadians());
        } else {
            outtakeSubsystem.stopTurret();
        }

        try {
            if (distanceLatch.unlocked()) {
                outtakeSubsystem.setHoodAngle(shooterDataTable.getSpecs(distanceLatch.open()).getAngle());
                throw new GoalNotFoundException(); //shortcut to latch reset
            }
        } catch (GoalNotFoundException e) {
            limelightSubsystem.addLatch(distanceLatch.reset());
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.stopTurret();
        outtakeSubsystem.setHoodAngle(defaultHoodAngle);
    }
}

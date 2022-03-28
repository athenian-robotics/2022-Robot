package frc.robot.commands.outtake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.*;
import static frc.robot.Constants.MechanismConstants.defaultHoodAngle;


public class AlwaysTurretTurnToGoalWithOdometryAndSetHoodAngleOrManualControl extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final OuttakeSubsystem outtakeSubsystem;
    private final ShooterDataTable shooterDataTable;
    private final LimelightDataLatch distanceLatch;

    public AlwaysTurretTurnToGoalWithOdometryAndSetHoodAngleOrManualControl(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem, OuttakeSubsystem outtakeSubsystem, ShooterDataTable shooterDataTable) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.outtakeSubsystem = outtakeSubsystem;
        this.shooterDataTable = shooterDataTable;

        distanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 5);

        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.addLatch(distanceLatch.reset());
    }

    @Override
    public void execute() {
        if (FightStick.fightStickJoystick.getX() < -0.5) { //TURRET ADJUSTMENT FALCON
            outtakeSubsystem.turretRunning = false;
            outtakeSubsystem.bangBangRunning = false;
            outtakeSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.5) {
            outtakeSubsystem.turretRunning = false;
            outtakeSubsystem.bangBangRunning = false;
            outtakeSubsystem.turnTurret(turretTurnSpeed);
        }else if (FightStick.fightStickJoystick.getY() < -0.5) { //TURRET ADJUSTMENT FALCON
                outtakeSubsystem.turretRunning = false;
                outtakeSubsystem.bangBangRunning = false;
                outtakeSubsystem.turnTurret(-slowTurretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() > 0.5) {
                outtakeSubsystem.turretRunning = false;
                outtakeSubsystem.bangBangRunning = false;
                outtakeSubsystem.turnTurret(slowTurretTurnSpeed);
        } else if (FightStick.fightStickShare.get()) {
            Transform2d goalVector = drivetrainSubsystem.getPose().minus(new Pose2d(8.25, 4.15, new Rotation2d(0)));
            outtakeSubsystem.setTurretPositionRadians(Math.toRadians(drivetrainSubsystem.getGyroAngle()) - Math.atan2(goalVector.getY(), goalVector.getX()));
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

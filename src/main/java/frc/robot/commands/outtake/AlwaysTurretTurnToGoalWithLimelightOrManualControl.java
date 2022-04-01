package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.MechanismConstants.slowTurretTurnSpeed;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;


public class AlwaysTurretTurnToGoalWithLimelightOrManualControl extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final LimelightDataLatch offsetLatch;

    public AlwaysTurretTurnToGoalWithLimelightOrManualControl(LimelightSubsystem limelightSubsystem, ShooterSubsystem shooterSubsystem) {
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
        if (FightStick.fightStickJoystick.getX() < -0.5) { //TURRET ADJUSTMENT FALCON
            shooterSubsystem.turretRunning = false;
            shooterSubsystem.bangBangRunning = false;
            shooterSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.5) {
            shooterSubsystem.turretRunning = false;
            shooterSubsystem.bangBangRunning = false;
            shooterSubsystem.turnTurret(turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() < -0.5) { //TURRET ADJUSTMENT FALCON
            shooterSubsystem.turretRunning = false;
            shooterSubsystem.bangBangRunning = false;
            shooterSubsystem.turnTurret(-slowTurretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() > 0.5) {
            shooterSubsystem.turretRunning = false;
            shooterSubsystem.bangBangRunning = false;
            shooterSubsystem.turnTurret(slowTurretTurnSpeed);
        } else if (FightStick.fightStickShare.getAsBoolean()) {
            try {
                if (offsetLatch.unlocked()) {
                    shooterSubsystem.setTurretPositionRadians(offsetLatch.open() + shooterSubsystem.getTurretAngleRadians());
                    throw new GoalNotFoundException(); //shortcut to latch reset  vvv  (since we've expended it)
                }
            } catch (GoalNotFoundException e) {
                limelightSubsystem.addLatch(offsetLatch.reset()); //assuming we want to look for the goal forever
            }
        } else {
            shooterSubsystem.stopTurret();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopTurret();
    }
}

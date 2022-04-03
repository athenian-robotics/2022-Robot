package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;


public class AlwaysTurretTurnToGoalWithOdometry extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final TurretSubsystem turretSubsystem;

    public AlwaysTurretTurnToGoalWithOdometry(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem, TurretSubsystem turretSubsystem, ShooterDataTable shooterDataTable) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.turretSubsystem = turretSubsystem;
        addRequirements(this.turretSubsystem);
    }

    @Override
    public void execute() {
        if (FightStick.fightStickShare.get()) {
            Transform2d goalVector = drivetrainSubsystem.getPose().minus(new Pose2d(8.25, 4.15, new Rotation2d(0)));
            turretSubsystem.setTurretSetpointRadians(Math.toRadians(drivetrainSubsystem.getGyroAngle()) - Math.atan2(goalVector.getY(), goalVector.getX()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.disable();
    }
}

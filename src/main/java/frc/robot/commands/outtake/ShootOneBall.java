package frc.robot.commands.outtake;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootTopBall;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.limelight.GuaranteeLimelightData;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.lib.GoalNotFoundException;
import frc.robot.subsystems.*;

public class ShootOneBall extends SequentialCommandGroup {
    public ShootOneBall(DrivetrainSubsystem drivetrain, IndexerSubsystem indexer, IntakeSubsystem intake, LimelightSubsystem limelight, OuttakeSubsystem outtake) {
        addRequirements(drivetrain, indexer, intake, limelight, outtake);
        try {
            addCommands(
                    new DisableIntake(intake),
                    new DisableDrivetrain(drivetrain),
                    new EnableShooter(outtake),
                    new ParallelDeadlineGroup(new GuaranteeLimelightData(limelight), new ManualAdjustTurret(outtake)),
                    new SetHoodAngle(outtake, limelight.getLimelightOutputAtIndex(2)),
                    new SetTurretActive(outtake, true),
                    new SetSpecificShooterPower(outtake, limelight.getLimelightOutputAtIndex(3)),
                    new GuaranteeLimelightDataEquals(limelight, 1, 0),
                    new SetTurretActive(outtake, false),
                    new ShootTopBall(indexer, 0.5),
                    new DisableShooter(outtake)
            );
        } catch (GoalNotFoundException ignored) {}
    }
}
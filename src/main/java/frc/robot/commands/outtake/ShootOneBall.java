package frc.robot.commands.outtake;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootTopBall;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.limelight.GuaranteeLimelightData;
import frc.robot.commands.limelight.GuaranteeLimelightDataEquals;
import frc.robot.lib.GoalNotFoundException;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class ShootOneBall extends SequentialCommandGroup {
    public ShootOneBall(DrivetrainSubsystem drivetrain, IndexerSubsystem indexer, IntakeSubsystem intake, LimelightSubsystem limelight, OuttakeSubsystem outtake, ShooterDataTable shooterDataTable) {
        addRequirements(drivetrain, indexer, intake, limelight, outtake);
        try {
            addCommands(
                    //Prepare
                    new DisableIntake(intake),
                    new DisableDrivetrain(drivetrain),
                    new EnableShooter(outtake),
                    new SetTurretStatus(outtake, false),
                    new SetHoodAngle(outtake, Constants.MechanismConstants.defaultHoodAngle),
                    //Aim and shoot once
                    new ParallelDeadlineGroup(new GuaranteeLimelightData(limelight), new ManualAdjustTurret(outtake)),
                    new SetHoodAngle(outtake, shooterDataTable.getSpecs(limelight.getLimelightOutputAtIndex(0)).getAngle()),
                    new SetTurretStatus(outtake, true),
                    new SetSpecificShooterPower(outtake, shooterDataTable.getSpecs(limelight.getLimelightOutputAtIndex(0)).getPower()),
                    new GuaranteeLimelightDataEquals(limelight, 1, 0),
                    new SetTurretStatus(outtake, false),
                    new ShootTopBall(indexer, intake, 0.5),
                    new DisableShooter(outtake)

            );
        } catch (GoalNotFoundException ignored) {
            // ඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞඞ
        }
    }
}
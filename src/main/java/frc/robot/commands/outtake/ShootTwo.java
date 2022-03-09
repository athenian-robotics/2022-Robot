package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.SetBothTelescopePositions;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootIndexedBallForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.commands.intake.RunIntakeWithoutPneumatics;
import frc.robot.commands.limelight.GuaranteeLimelightData;
import frc.robot.lib.GoalNotFoundException;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;


public class ShootTwo extends SequentialCommandGroup {
    public ShootTwo(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer, IntakeSubsystem intake, OuttakeSubsystem outtake, LimelightSubsystem limelight, ShooterDataTable shooterDataTable) {
        if (climber.getLeftHeightPercent() > 0.3 || climber.getRightHeightPercent() > 0.3) this.cancel();
        try {
            addCommands(
                    //Prepare
                    new DisableDrivetrain(drivetrain),
                    new SetBothTelescopePositions(climber, 0),
                    new DisableIntake(intake),
                    //Align to shoot
                    new ParallelDeadlineGroup(new GuaranteeLimelightData(limelight), new ManualAdjustTurret(outtake)),
                    new ParallelCommandGroup(
                            new TurretTurnToGoal(limelight, outtake),
                            new SetHoodAngle(outtake, shooterDataTable.getSpecs(limelight.getDistance()).getAngle()),
                            new SetShooterPower(outtake, shooterDataTable.getSpecs(limelight.getDistance()).getSpeed())),
                    //Shoot 1st
                    new ShootIndexedBallForever(indexer, intake, outtake).withTimeout(2.5),
                    //Index next ball (may or may not be there) TODO remove withTimeout() on RunIntakeWithoutPneumatics()
                    new RunIntakeWithoutPneumatics(intake, indexer).withTimeout(2.5),
                    //Shoot 2nd
                    new ShootIndexedBallForever(indexer, intake, outtake).withTimeout(2.5),
                    //Return to teleop
                    new SetShooterPower(outtake, 0),
                    new SetHoodAngle(outtake, 8)
            );
        } catch (GoalNotFoundException e) {
            System.out.println("Goal not found for ShootTwo!");
        }
    }
}
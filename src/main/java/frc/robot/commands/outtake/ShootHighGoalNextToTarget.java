package frc.robot.commands.outtake;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.indexer.ShootIndexedBallsForever;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.subsystems.*;

public class ShootHighGoalNextToTarget extends SequentialCommandGroup {
    public ShootHighGoalNextToTarget(ClimberSubsystem climber, DrivetrainSubsystem drivetrain,
                                     IndexerSubsystem indexer, IntakeSubsystem intake, OuttakeSubsystem outtake,
                                     LimelightSubsystem limelight) {
        if (climber.getLeftHeightPercent() > 0.1 || climber.getRightHeightPercent() > 0.1) this.cancel();
        //Prepare
        addCommands(
                new DisableDrivetrain(drivetrain),
                new DisableIntake(intake),
                //Align to shoot
                new SetShooterPower(outtake, 36),
                new SetHoodAngleTimeSafe(outtake, 18.5),
                new WaitCommand(0.7),
                //Shoot Balls
                new ShootIndexedBallsForever(indexer, intake).withTimeout(2),
                //Return to teleop
                new ParallelCommandGroup(
                        new DisableShooter(outtake),
                        new SetHoodAngle(outtake, Constants.MechanismConstants.defaultHoodAngle)
                )
        );
    }
}
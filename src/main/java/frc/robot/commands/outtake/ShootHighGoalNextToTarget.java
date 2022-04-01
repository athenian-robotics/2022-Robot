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
    public ShootHighGoalNextToTarget(DrivetrainSubsystem drivetrain, IndexerSubsystem indexer,
                                     IntakeSubsystem intake, ShooterSubsystem outtake, PortalSubsystem portal) {
        addCommands(
                new DisableDrivetrain(drivetrain),
                new DisableIntake(intake),
                //Align to shoot
                new SetShooterPower(outtake, 36),
                new SetHoodAngleTimeSafe(outtake, 18.5),
                new WaitCommand(0.7),
                //Shoot Balls
                new ShootIndexedBallsForever(indexer, intake, portal).withTimeout(2),
                //Return to teleop
                new ParallelCommandGroup(
                        new DisableShooter(outtake),
                        new SetHoodAngle(outtake, Constants.MechanismConstants.defaultHoodAngle)
                )
        );
    }
}
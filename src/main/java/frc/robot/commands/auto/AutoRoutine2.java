package frc.robot.commands.auto;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoRoutine2Contents.AutoRoutine2Part1;
import frc.robot.commands.auto.AutoRoutine2Contents.AutoRoutine2Part2;
import frc.robot.commands.auto.AutoRoutine2Contents.AutoRoutine2Part3;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.outtake.AlwaysTurretTurnToGoalWithLimelightOrManualControl;
import frc.robot.commands.outtake.ShootOne;
import frc.robot.commands.outtake.ShootTwo;
import frc.robot.commands.outtake.TurretTurnToAngle;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

public class AutoRoutine2 extends SequentialCommandGroup {
    public AutoRoutine2(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer,
                        IntakeSubsystem intake, OuttakeSubsystem outtake, LimelightSubsystem limelight,
                        ShooterDataTable shooterDataTable) {
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                new ToggleIntake(intake),
                                new AutoRoutine2Part1(drivetrain), //drives up to ball
                                new ToggleIntake(intake),
                                new ShootTwo(climber, drivetrain, indexer, intake, outtake, limelight, shooterDataTable),
                                //new ShootBalls(climber, drivetrain, indexer, intake, outtake, limelight, shooterDataTable),
                                new AutoRoutine2Part2(drivetrain) //drives to the right to get another ball
                        ), new AlwaysTurretTurnToGoalWithLimelightOrManualControl(limelight, outtake)
                        )
        );
    }
}
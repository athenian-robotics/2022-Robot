package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoRoutine1Contents.*;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.outtake.AlwaysTurretTurnToGoalWithLimelightOrManualControl;
import frc.robot.commands.outtake.ShootTwoWithoutTurret;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;


public class AutoRoutine1 extends SequentialCommandGroup {
    public AutoRoutine1(ClimberSubsystem climber, DrivetrainSubsystem drivetrain, IndexerSubsystem indexer,
                        IntakeSubsystem intake, OuttakeSubsystem outtake, LimelightSubsystem limelight,
                        ShooterDataTable shooterDataTable) {
        addCommands(
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new ToggleIntake(intake),
                            new AutoRoutine1Part1(drivetrain), //moves left to other ball
                            //moves down to ball under starting position
                            new ToggleIntake(intake),
                            new ShootTwoWithoutTurret(climber, drivetrain, indexer, intake, outtake, limelight, shooterDataTable),
                            new AutoRoutine1Part2(drivetrain), // turn back
                                new ToggleIntake(intake),
                                new AutoRoutine1Part25(drivetrain), // intake other ball
                            new AutoRoutine1Part3(drivetrain), //moves left to other ball
                            //drives to human player terminal
                            new WaitCommand(.75),
                            new ToggleIntake(intake),
                            new AutoRoutine1Part4(drivetrain), //drives back closer to the goal
                            new ShootTwoWithoutTurret(climber, drivetrain, indexer, intake, outtake, limelight, shooterDataTable)
                            ),
                        new AlwaysTurretTurnToGoalWithLimelightOrManualControl(limelight, outtake)
                )
        );
    }
}
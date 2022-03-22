package frc.robot.commands.auto;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.AutoRoutine0Contents.AutoRoutine0Part1;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.outtake.ShootTwo;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;


public class AutoRoutine0 extends SequentialCommandGroup {
    public AutoRoutine0(DrivetrainSubsystem drivetrain){
        addCommands(
                new AutoRoutine6(drivetrain)
        );
    }
}
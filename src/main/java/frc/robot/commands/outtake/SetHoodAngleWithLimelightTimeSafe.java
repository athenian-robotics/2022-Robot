package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.RobotContainer.limelight;
import static frc.robot.RobotContainer.outtake;


public class SetHoodAngleWithLimelightTimeSafe extends SequentialCommandGroup {
    public SetHoodAngleWithLimelightTimeSafe(ShooterDataTable shooterDataTable, LimelightSubsystem limelightSubsystem, OuttakeSubsystem outtakeSubsystem) {
        super(
                new SetHoodAngleWithLimelight(shooterDataTable, limelight, outtake),
                new WaitCommand((Math.abs(outtake.getHoodAngle() - Constants.MechanismConstants.defaultHoodAngle)/6.8) - 1.4)
        ); //We have to take the L with this calculation and assume that the hood angle starts at the default instead of taking a measurement
    }
}
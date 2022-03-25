package frc.robot.commands.outtake;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class AlwaysSetHoodAngleAndTurret extends ParallelCommandGroup {
    public AlwaysSetHoodAngleAndTurret(LimelightSubsystem limelight, OuttakeSubsystem outtake, ShooterDataTable shooterDataTable) {
        addCommands(
                new AlwaysTurretTurnToGoalWithLimelightOrManualControl(limelight, outtake),
                new SetHoodAngleWithLimelight(shooterDataTable, limelight, outtake)
        );
    }
}
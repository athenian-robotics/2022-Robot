package frc.robot.commands.outtake;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DisableDrivetrain;
import frc.robot.commands.intake.DisableIntake;
import frc.robot.subsystems.*;

public class ShootOneBall extends SequentialCommandGroup {
    public ShootOneBall(DrivetrainSubsystem drivetrain, IndexerSubsystem indexer, IntakeSubsystem intake, LimelightSubsystem limelight, OuttakeSubsystem outtake) {
        super(
                new DisableIntake(intake),
                new DisableDrivetrain(drivetrain),
                new EnableShooter(outtake)
        );
    }
}
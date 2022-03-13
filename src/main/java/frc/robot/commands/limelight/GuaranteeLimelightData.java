package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;


public class GuaranteeLimelightData extends CommandBase {
    private final LimelightSubsystem limelightSubsystem;

    public GuaranteeLimelightData(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

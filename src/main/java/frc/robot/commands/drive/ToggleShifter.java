package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class ToggleShifter extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    public ToggleShifter(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(this.drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.tankDrive(0, 0);
        drivetrainSubsystem.toggleShifter();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

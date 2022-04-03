package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;


public class ToggleShifter extends InstantCommand {
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
}

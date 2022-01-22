package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class TankDrive extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final XboxController xboxController;

    public TankDrive(DrivetrainSubsystem drivetrain, XboxController xboxController) {
        this.drivetrain = drivetrain;
        this.xboxController = xboxController;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        drivetrain.tankDrive(xboxController.getLeftY(), // Left velocity
                             -xboxController.getRightY()); // Right velocity
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ArcadeDrive extends CommandBase {
    // Define xbox controller and drivetrain subsystem
    private final DrivetrainSubsystem drivetrain;
    private final XboxController controller;

    public ArcadeDrive(DrivetrainSubsystem drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {} // Nothing needed for setup

    @Override
    public void execute() {
        drivetrain.arcadeDrive(controller.getLeftY(), // Throttle (Xbox Controller: Left stick, Y axis)
                -controller.getRightX()); // Rotation (Xbox controller: Right stick, X axis)
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}

package frc.robot.commands.auto.components;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.DriveConstants.maxAutoSpeed;

public class AutoForwardDistance extends CommandBase {
    private final double metersToDrive;
    private final double Kp = 3.3; //3.3
    private final double Ki = 0.0;
    private final double Kd = 0.13; //0.13
    // Setup PID variables
    DrivetrainSubsystem drivetrain;
    PIDController pid = new PIDController(Kp, Ki, Kd);
    private double setpoint = 0.0;
    private double drivePower = 0.0;

    public AutoForwardDistance(DrivetrainSubsystem drivetrainSubsystem, double metersToDrive) {
        this.metersToDrive = metersToDrive;
        this.drivetrain = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        this.setpoint = drivetrain.getRightDistanceDriven() + metersToDrive; // Configure PID setpoint and tolerance
        pid.setTolerance(0.001);
        pid.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        int sign = pid.calculate(drivetrain.getRightDistanceDriven()) < 0 ? -1 : 1; // Directional
        this.drivePower = Math.min(Math.abs(pid.calculate(drivetrain.getRightDistanceDriven())), maxAutoSpeed); //
        // Use pid to calculate power
        drivePower *= sign; // Negate necessary sign
        System.out.println(drivePower); // Debugging purposes
        drivetrain.tankDrive(drivePower, drivePower); // Drive with calculated power
    }

    @Override
    public boolean isFinished() {return pid.atSetpoint();} // End if we are at the setpoint

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    } // Stop driving
}

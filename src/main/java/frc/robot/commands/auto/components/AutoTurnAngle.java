package frc.robot.commands.auto.components;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTurnAngle extends CommandBase {
    // Setup PID Variables
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final double angleToTurn;
    private double power = 0.0;
    private double setpoint = 0.0;
    private final double Kp = 0.01;
    private final double Ki = 0.0;
    private final double Kd = 0.0001;

    final PIDController pid = new PIDController(Kp, Ki, Kd);

    public AutoTurnAngle(DrivetrainSubsystem drivetrainSubsystem, double angleToTurn) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        double tolerance = 0.2;
        this.angleToTurn = angleToTurn;
        pid.setTolerance(tolerance);
        addRequirements(this.drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        setpoint = drivetrainSubsystem.getHeading() + angleToTurn; // Configure pid tolerance and setpoint
        pid.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        power = Math.min(Math.abs(pid.calculate(drivetrainSubsystem.getGyroYaw())),
                Constants.AutoConstants.maxAutoTurn); // Scale under max constant
        int leftSign = Math.abs(setpoint - drivetrainSubsystem.getGyroYaw()) > 180 ? -1 : 1; // Fastest direction
        int rightSign = Math.abs(setpoint - drivetrainSubsystem.getGyroYaw()) > 180 ? 1 : -1; // Fastest direction
        power = Math.abs(power) < 0.01 ? 0 : power; // Deadband
        System.out.println(power); // Debugging purposes
        drivetrainSubsystem.autoTankDrive(power * leftSign, power * rightSign); // Drive with calculated power
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || power < 0.01;
    } // End if at setpoint or too low power

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.autoTankDrive(0, 0);
    } // Disable motors
}

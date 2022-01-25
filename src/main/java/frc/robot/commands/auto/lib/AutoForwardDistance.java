package frc.robot.commands.auto.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.DriveConstants.maxAutoSpeed;

public class AutoForwardDistance extends CommandBase {
    DrivetrainSubsystem drivetrain;
    private final double metersToDrive;
    private final double Kp = 3.3; //3.3
    private final double Ki = 0.0;
    private final double Kd = 0.13; //0.13
    private double setpoint = 0.0;
    private double drivePower = 0.0;
    PIDController pid = new PIDController(Kp, Ki, Kd);

    public AutoForwardDistance(DrivetrainSubsystem drivetrainSubsystem, double metersToDrive) {
        this.metersToDrive = metersToDrive;
        this.drivetrain = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        this.setpoint = drivetrain.getRightDistanceDriven() + metersToDrive;
        pid.setTolerance(0.001);
        pid.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        int sign = pid.calculate(drivetrain.getRightDistanceDriven()) < 0 ? -1 : 1;
        this.drivePower = Math.min(Math.abs(pid.calculate(drivetrain.getRightDistanceDriven())), maxAutoSpeed);
        drivePower *= sign;
        System.out.println(drivePower);
        drivetrain.tankDrive(drivePower, drivePower);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint(); // || Math.abs(drivePower) < 0.26;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }}

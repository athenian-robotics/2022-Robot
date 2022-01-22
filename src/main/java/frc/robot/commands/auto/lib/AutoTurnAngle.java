package frc.robot.commands.auto.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;


public class AutoTurnAngle extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;
    private int leftSign;
    private int rightSign;
    private long startTime;
    private long elapsedTime;
    private final double tolerance;
    private final double angleToTurn;
    private double power = 0.0;
    private double setpoint = 0.0;
    private double Kp = 0.0;
    private double Ki = 0.0;
    private double Kd = 0.0;

    PIDController pid = new PIDController(Kp, Ki, Kd);

    public AutoTurnAngle(DrivetrainSubsystem drivetrainSubsystem, double angleToTurn) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.tolerance = 0.5;
        this.angleToTurn = angleToTurn;
        pid.setTolerance(tolerance);
        addRequirements(this.drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        setpoint = drivetrainSubsystem.getGyroAngle() + angleToTurn;
        pid.setSetpoint(setpoint);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        elapsedTime = System.currentTimeMillis() - startTime;
        power = Math.min(pid.calculate(drivetrainSubsystem.getGyroAngle()), Constants.DriveConstants.maxAutoSpeed);
        leftSign = Math.abs(setpoint - drivetrainSubsystem.getGyroAngle()) > 180 ? -1 : 1;
        rightSign = Math.abs(setpoint - drivetrainSubsystem.getGyroAngle()) > 180 ? 1 : -1;

        drivetrainSubsystem.autoTankDrive(power * leftSign, power * rightSign);
    }

    @Override
    public boolean isFinished() {
        return drivetrainSubsystem.getGyroAngle() <= setpoint + tolerance && drivetrainSubsystem.getGyroAngle() >= setpoint - tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.autoTankDrive(0, 0);
    }
}

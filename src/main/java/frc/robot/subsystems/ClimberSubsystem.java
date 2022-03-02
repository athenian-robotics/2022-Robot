package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MechanismConstants.leftClimberMotorPort;
import static frc.robot.Constants.MechanismConstants.rightClimberMotorPort;


public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climbMotorLeft = new TalonFX(leftClimberMotorPort);
    private final TalonFX climbMotorRight = new TalonFX(rightClimberMotorPort);

    private final PIDController leftPIDController;
    private final PIDController rightPIDController;

    public boolean climberActive = false;

    public ClimberSubsystem() {
        climbMotorLeft.setInverted(true);
        climbMotorRight.setInverted(false);

        climbMotorLeft.setNeutralMode(NeutralMode.Brake);
        climbMotorRight.setNeutralMode(NeutralMode.Brake);
        climbMotorLeft.setSelectedSensorPosition(0);
        climbMotorRight.setSelectedSensorPosition(0);

        leftPIDController = new PIDController(0, 0, 0);
        rightPIDController = new PIDController(0, 0, 0);
        leftPIDController.setTolerance(0.01);
        rightPIDController.setTolerance(0.01);
        leftPIDController.setSetpoint(0);
        rightPIDController.setSetpoint(0);
    }

    public void setLeftMotor(double power) {
        climbMotorLeft.set(ControlMode.PercentOutput, power);
    }

    public void setRightMotor(double power) {
        climbMotorRight.set(ControlMode.PercentOutput, power);
    }

    public void setLeftHeightPercent(double percent) {
        if (percent < 0 || percent > 1) return;
        leftPIDController.setSetpoint(percent);
    }

    public void setRightHeightPercent(double percent) {
        if (percent < 0 || percent > 1) return;
        rightPIDController.setSetpoint(percent);
    }

    private double getLeftHeightEncoderCount() {
        return climbMotorLeft.getSelectedSensorPosition();
    }

    private double getRightHeightEncoderCount() {
        return climbMotorRight.getSelectedSensorPosition();
    }

    public double getLeftHeightPercent() {
        return getLeftHeightEncoderCount()/297214;
    }

    public double getRightHeightPercent() {
        return getRightHeightEncoderCount()/297214;
    }

    public void disable() {
        climberActive = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left telescope height", getLeftHeightPercent());
        SmartDashboard.putNumber("right telescope height", getRightHeightPercent());

        if (climberActive) {
            setLeftMotor(leftPIDController.calculate(getLeftHeightPercent()));
            setRightMotor(rightPIDController.calculate(getRightHeightPercent()));
        } else {
            setLeftMotor(0);
            setRightMotor(0);
        }
    }
}
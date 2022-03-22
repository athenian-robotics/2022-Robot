package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import static frc.robot.Constants.MechanismConstants.*;


public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climbMotorLeft = new TalonFX(leftClimberMotorPort);
    private final TalonFX climbMotorRight = new TalonFX(rightClimberMotorPort);
    private final TalonFX climbWinchMotor = new TalonFX(climbWinchMotorPort);

    private NetworkTableEntry climbPercentNTE;

    public boolean climberActive = false;

    public ClimberSubsystem() {
        climbMotorLeft.setInverted(true);
        climbMotorRight.setInverted(false);
        climbWinchMotor.setInverted(true);

        climbMotorLeft.setNeutralMode(NeutralMode.Brake);
        climbMotorRight.setNeutralMode(NeutralMode.Brake);
        climbWinchMotor.setNeutralMode(NeutralMode.Brake);
        climbMotorLeft.setSelectedSensorPosition(0);
        climbMotorRight.setSelectedSensorPosition(0);
        climbWinchMotor.setSelectedSensorPosition(0);

        climbPercentNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Climber Percent", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1))
                .getEntry();
    }

    public void setLeftMotor(double percent) {
        climbMotorLeft.set(ControlMode.PercentOutput, percent);
    }

    public void setRightMotor(double percent) {
        climbMotorRight.set(ControlMode.PercentOutput, percent);
    }

    public void setWinch(double percent) {
        climbWinchMotor.set(ControlMode.PercentOutput, percent);
    }

    private double getLeftHeightEncoderCount() {
        return climbMotorLeft.getSelectedSensorPosition();
    }

    private double getRightHeightEncoderCount() {
        return climbMotorRight.getSelectedSensorPosition();
    }

    private double getWinchEncoderCount(){
        return climbWinchMotor.getSelectedSensorPosition();
    }

    public double getLeftHeightPercent() {
        return getLeftHeightEncoderCount() / leftClimberMaxEncoderCount;
    }

    public double getRightHeightPercent() {
        return getRightHeightEncoderCount() / rightClimberMaxEncoderCount;
    }

    public double getWinchPercent() {
        return getWinchEncoderCount() / winchMaxEncoderCount;
    }

    public void disable() {
        climberActive = false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left telescope height", getLeftHeightPercent());
        SmartDashboard.putNumber("right telescope height", getRightHeightPercent());
        SmartDashboard.putNumber("left telescope encoder count", getLeftHeightEncoderCount());
        SmartDashboard.putNumber("right telescope encoder count", getRightHeightEncoderCount());
        SmartDashboard.putNumber("Winch", getWinchEncoderCount());
        SmartDashboard.putBoolean("Climb Active", climberActive);
    }

    public void set(int i) {
        // set motors to encoder count

        climbMotorLeft.set(ControlMode.Position, i);
    }
}
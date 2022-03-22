package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.climb.SetBothTelescopePositions;

import java.util.Map;

import static frc.robot.Constants.MechanismConstants.*;


public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climbMotorLeft = new TalonFX(leftClimberMotorPort);
    private final TalonFX climbMotorRight = new TalonFX(rightClimberMotorPort);
    private final TalonFX climbWinchMotor = new TalonFX(climbWinchMotorPort);

    private final PIDController leftPIDController;
    private final PIDController rightPIDController;
    private final PIDController winchPIDController;

    private NetworkTableEntry climbPercentNTE;

    public boolean climberActive = false;

    public ClimberSubsystem() {
        climbMotorLeft.setInverted(true);
        climbMotorRight.setInverted(false);

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

        leftPIDController = new PIDController(0, 0, 0);
        rightPIDController = new PIDController(0, 0, 0);
        winchPIDController = new PIDController(0,0,0);
        leftPIDController.setTolerance(0.01);
        rightPIDController.setTolerance(0.01);
        winchPIDController.setTolerance(0.01);
        leftPIDController.setSetpoint(0);
        rightPIDController.setSetpoint(0);
        winchPIDController.setSetpoint(0);
    }

    public void setLeftMotor(double ticks) {
        //climbMotorLeft.set(ControlMode.Position, ticks*269578);
        climbMotorLeft.set(ControlMode.PercentOutput, ticks);
    }

    public void setLeftPercent(double position){
        if(position < 0 || position >1) return;
        leftPIDController.setSetpoint(position);
    }

    public void setRightMotor(double power) {
        //climbMotorRight.set(ControlMode.Position, power*269578);
        climbMotorRight.set(ControlMode.PercentOutput, power);
    }

    public void setRightPercent(double position){
        if(position < 0|| position >1) return;
        rightPIDController.setSetpoint(position);
    }

    public void setWinch(double power){
        climbWinchMotor.set(ControlMode.Position, power*269578);
    }

    public void setWinchPower(double power) {
        climbWinchMotor.set(ControlMode.PercentOutput, power);
    }

    public void setWinchPosition(double position){
        winchPIDController.setSetpoint(position);
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
        return getLeftHeightEncoderCount()/269578;
    }

    public double getRightHeightPercent() {
        return getRightHeightEncoderCount()/269578;
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
        //setLeftMotor(climbPercentNTE.getDouble(0));
        //setRightMotor(climbPercentNTE.getDouble(0));
    }
}
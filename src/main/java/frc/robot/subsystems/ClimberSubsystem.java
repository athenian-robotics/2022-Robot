package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MechanismConstants.leftClimberMotorPort;
import static frc.robot.Constants.MechanismConstants.rightClimberMotorPort;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX climbMotorLeft = new TalonFX(leftClimberMotorPort);
    private final TalonFX climbMotorRight = new TalonFX(rightClimberMotorPort);

    private final PIDController leftPIDController;
    private final PIDController rightPIDController;

    public ClimberSubsystem() {

        climbMotorLeft.setInverted(false);
        climbMotorRight.setInverted(true);

        leftPIDController = new PIDController(0, 0, 0);
        rightPIDController = new PIDController(0, 0, 0);
        leftPIDController.setTolerance(0.5);
        rightPIDController.setTolerance(0.5);

    }

    public void setLeftMotor(double power) {
        climbMotorLeft.set(ControlMode.PercentOutput, power);
    }

    public void setLeftPosition(double position) {
        leftPIDController.setSetpoint(position);
        climbMotorLeft.set(ControlMode.PercentOutput, leftPIDController.calculate(position));
    }

    public void setRightMotor(double power) {
        climbMotorRight.set(ControlMode.PercentOutput, power);
    }

    public void setRightPosition(int position) {
        rightPIDController.setSetpoint(position);
        climbMotorRight.set(ControlMode.PercentOutput, rightPIDController.calculate(position));
    }

    @Override
    public void periodic() {

    }
}


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

    public ClimberSubsystem() {
        climbMotorLeft.setInverted(true);
        climbMotorRight.setInverted(false);

        climbMotorLeft.setNeutralMode(NeutralMode.Brake);
        climbMotorRight.setNeutralMode(NeutralMode.Brake);

        leftPIDController = new PIDController(0, 0, 0);
        rightPIDController = new PIDController(0, 0, 0);
        leftPIDController.setTolerance(0.5);
        rightPIDController.setTolerance(0.5);
    }

    public void setLeftMotor(double power) {
        climbMotorLeft.set(ControlMode.PercentOutput, power);
    }

    public void setLeftPosition(double position) {
        //leftPIDController.setSetpoint(position);
    }

    public void setRightMotor(double power) {
        climbMotorRight.set(ControlMode.PercentOutput, power);
    }

    public void setRightPosition(double position) {
        //rightPIDController.setSetpoint(position);
    }

    public double getLeftHeightMeters() {
        return 0.0254 * ((33.5 * climbMotorLeft.getSelectedSensorPosition() / 287578) + 33.5);
    }

    public double getRightHeightMeters() {
        return 0.0254 * ((33.5 * climbMotorRight.getSelectedSensorPosition() / 287578) + 33.5);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left telescope height", getLeftHeightMeters());
        SmartDashboard.putNumber("right telescope height", getRightHeightMeters());
    }
}


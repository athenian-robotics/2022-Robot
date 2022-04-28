package frc.robot.subsystems;

import static frc.robot.Constants.MechanismConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climbMotorLeft = new TalonFX(leftClimberMotorPort);
  private final TalonFX climbMotorRight = new TalonFX(rightClimberMotorPort);
  private final TalonFX climbWinchMotor = new TalonFX(climbWinchMotorPort);

  public ClimberSubsystem() {
    climbMotorLeft.setInverted(true);
    climbMotorRight.setInverted(false);
    climbWinchMotor.setInverted(false);

    climbMotorLeft.setNeutralMode(NeutralMode.Brake);
    climbMotorRight.setNeutralMode(NeutralMode.Brake);
    climbWinchMotor.setNeutralMode(NeutralMode.Brake);
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

  public void disable() {
    setLeftMotor(0);
    setRightMotor(0);
  }

}
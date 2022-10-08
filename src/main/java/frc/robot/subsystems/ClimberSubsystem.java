package frc.robot.subsystems;

import static frc.robot.Constants.MechanismConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climbMotorLeft = new TalonFX(leftClimberMotorPort);
  private final TalonFX climbMotorRight = new TalonFX(rightClimberMotorPort);

  public ClimberSubsystem() {
    climbMotorLeft.setInverted(true);
    climbMotorRight.setInverted(false);

    climbMotorLeft.setNeutralMode(NeutralMode.Brake);
    climbMotorRight.setNeutralMode(NeutralMode.Brake);
  }

  private void setTelescopeSpeed(double percent) {
    climbMotorLeft.set(ControlMode.PercentOutput, percent);
    climbMotorRight.set(ControlMode.PercentOutput, percent);
  }

  public Command telescopeUp() {
    return new StartEndCommand(() -> setTelescopeSpeed(.5), () -> setTelescopeSpeed(0));
  }

  public Command telescopeDown() {
    return new StartEndCommand(() -> setTelescopeSpeed(-.5), () -> setTelescopeSpeed(0));
  }
}

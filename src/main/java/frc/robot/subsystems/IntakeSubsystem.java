package frc.robot.subsystems;

import static frc.robot.Constants.PneumaticConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
  // Configure intake motor, solenoid, and booleans
  private final TalonFX intakeMotor = new TalonFX(Constants.MechanismConstants.intakeMotorPort);

  private final DoubleSolenoid rightIntakePneumatic =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortRightA, pneumaticPortRightB);
  private final DoubleSolenoid leftIntakePneumatic =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortLeftA, pneumaticPortLeftB);

  @Log.ToString private IntakeState state = IntakeState.IDLE_RETRACTED;

  public enum IntakeState {
    SPIT_EXTENDED,
    SUCK_EXTENDED,
    SUCK_RETRACTED,
    IDLE_RETRACTED
  }

  public IntakeSubsystem() {
    intakeMotor.configFactoryDefault(); // Initialize motor set up
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    intakeMotor.configOpenloopRamp(0.1); // Ramp up (Trapezoid)
    intakeMotor.configClosedloopRamp(0.1); // Ramp down (Trapezoid)
  }

  public void startIntake() { // Enables intake
    intakeMotor.set(ControlMode.PercentOutput, Constants.MechanismConstants.intakeSpeed);
  }

  private void stopIntake() { // Disables intake
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  private void startIntakeInverted() { // Inverts intake wheel direction
    intakeMotor.set(ControlMode.PercentOutput, -Constants.MechanismConstants.intakeSpeed);
  }

  private void extendPneumatic() { // Extends pneumatic
    rightIntakePneumatic.set(DoubleSolenoid.Value.kForward);
    leftIntakePneumatic.set(DoubleSolenoid.Value.kForward);
  }

  private void retractPneumatic() { // Retracts pneumatic
    rightIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
    leftIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
  }

  public Command suckExtended() {
    return new InstantCommand(
        () -> {
          state = IntakeState.SUCK_EXTENDED;
          RobotContainer.portal.startPortal();
          extendPneumatic();
          startIntake();
        });
  }

  public Command spitExtended() {
    return new InstantCommand(
        () -> {
          state = IntakeState.SPIT_EXTENDED;
          extendPneumatic();
          startIntakeInverted();
        });
  }

  public Command suckRetracted() {
    return new InstantCommand(
        () -> {
          RobotContainer.portal.stopPortal();
          state = IntakeState.SUCK_RETRACTED;
          retractPneumatic();
          startIntake();
        });
  }

  public Command idleRetracted() {
    return new InstantCommand(
        () -> {
          RobotContainer.portal.run2();
          state = IntakeState.IDLE_RETRACTED;
          retractPneumatic();
          stopIntake();
        });
  }
}

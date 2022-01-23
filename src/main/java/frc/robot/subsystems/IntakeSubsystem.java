package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.PneumaticConstants.*;


public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(Constants.MechanismConstants.intakeMotorPort);

    private final DoubleSolenoid leftIntakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, intakePneumaticPortLeftA, intakePneumaticPortLeftB);
    private final DoubleSolenoid rightIntakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortRightA, pneumaticPortRightB);

    public boolean isRunning = false;
    public boolean isExtended = false;
    public boolean isInverted = false;

    public IntakeSubsystem() {
        leftIntakePneumatic.close();
        rightIntakePneumatic.close();

        intakeMotor.configFactoryDefault(); // Initialize motor set up
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.configOpenloopRamp(0.1); // Ramp up (Trapezoid)
        intakeMotor.configClosedloopRamp(0.1); // Ramp down (Trapezoid)
    }

    private void startIntake() {
        intakeMotor.set(ControlMode.PercentOutput, Constants.MechanismConstants.intakeSpeed);
        isRunning = true;
    }

    private void stopIntake() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
        isRunning = false;
        isInverted = false;
    }

    private void toggleIntake() {
        if (isRunning) {
            stopIntake();
        } else {
            startIntake();
        }
    }

    public void toggleIntakeAndPneumatic() {
        togglePneumatic(); toggleIntake();
        if (!isExtended && isRunning) disable();
    }

    public void invert() {
        isInverted = !this.isInverted;
        int sign = isInverted ? 1: -1;
        if (isRunning) intakeMotor.set(ControlMode.PercentOutput, sign * Constants.MechanismConstants.intakeSpeed);
    }

    private void extendPneumatic() {
        rightIntakePneumatic.set(DoubleSolenoid.Value.kForward);
        leftIntakePneumatic.set(DoubleSolenoid.Value.kForward);
        isExtended = true;
    }

    private void retractPneumatic() {
        rightIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
        leftIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
        isExtended = false;
    }

    private void togglePneumatic(){
        if (rightIntakePneumatic.get() != leftIntakePneumatic.get()) {
            return;
        } if (rightIntakePneumatic.get() == DoubleSolenoid.Value.kForward) {
            retractPneumatic();
        } else {
            extendPneumatic();
        }
    }


    public void disable() {
        stopIntake();
        retractPneumatic();
    }

    @Override
    public void periodic(){}
}


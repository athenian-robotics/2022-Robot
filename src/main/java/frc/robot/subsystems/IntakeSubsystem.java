package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    public IntakeSubsystem() {
        leftIntakePneumatic.close();
        rightIntakePneumatic.close();
    }

    public void startIntake() {
        intakeMotor.set(ControlMode.PercentOutput, Constants.MechanismConstants.intakeSpeed);
        isRunning = true;
    }

    public void stopIntake() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }

    public void toggleIntake() {
        if (isRunning) {
            stopIntake();
        } else {
            startIntake();
        }
    }

    public void invert() {
        intakeMotor.setInverted(!intakeMotor.getInverted());
    }

    public void extendPneumatic() {
        rightIntakePneumatic.set(DoubleSolenoid.Value.kForward);
        leftIntakePneumatic.set(DoubleSolenoid.Value.kForward);
        isExtended = true;
    }

    public void retractPneumatic() {
        rightIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
        leftIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
        isExtended = false;
    }

    public void togglePneumatic(){
        if (rightIntakePneumatic.get() != leftIntakePneumatic.get()) {
            return;
        } if (rightIntakePneumatic.get() == DoubleSolenoid.Value.kForward) {
            retractPneumatic();
        } else {
            extendPneumatic();
        }
    }

    @Override
    public void periodic(){}
}


package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MechanismConstants.intakeMotorPort;
import static frc.robot.Constants.PneumaticConstants.*;


public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(intakeMotorPort);
    private final DoubleSolenoid intakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, intakePneumaticPortA, intakePneumaticPortB);
    private final DoubleSolenoid testPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortOneA, pneumaticPortOneB);
    public boolean isRunning = false;
    public boolean isExtended = false;

    public IntakeSubsystem() {
        intakePneumatic.close();
        testPneumatic.close();
    }

    public void startIntake() {
        intakeMotor.set(ControlMode.PercentOutput, 0.5);
        isRunning = true;
    }

    public void invert() {
        intakeMotor.setInverted(!intakeMotor.getInverted());
    }

    public void stopIntake() {
        intakeMotor.set(ControlMode.PercentOutput,0);
        isRunning = false;
    }

    public void toggleIntake() {
        if (isRunning) {
            stopIntake();
        } else {
            startIntake();
        }
    }

    public void togglePneumatic(){
        intakePneumatic.toggle();
    }

    @Override
    public void periodic(){}
}


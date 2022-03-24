package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.MechanismConstants.intakeToIndexerMotorPort;
import static frc.robot.Constants.MechanismConstants.intakeToIndexerSpeed;
import static frc.robot.Constants.PneumaticConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    // Configure intake motor, solenoid, and booleans
    private final TalonFX intakeMotor = new TalonFX(Constants.MechanismConstants.intakeMotorPort);

    private final CANSparkMax intakeToIndexerMotor = new CANSparkMax(intakeToIndexerMotorPort,
            CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DoubleSolenoid rightIntakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            pneumaticPortRightA, pneumaticPortRightB);
    private final DoubleSolenoid leftIntakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            pneumaticPortLeftA, pneumaticPortLeftB);

    public boolean isRunning = false;
    public boolean isExtended = false;
    public boolean isInverted = false;
    public final boolean intakeToIndexerRunning = false;

    public IntakeSubsystem() {
        intakeMotor.configFactoryDefault(); // Initialize motor set up
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.configOpenloopRamp(0.1); // Ramp up (Trapezoid)
        intakeMotor.configClosedloopRamp(0.1); // Ramp down (Trapezoid)
        intakeToIndexerMotor.setInverted(true);

        NetworkTableEntry intakeNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Intake Active", false)
                .getEntry();
    }

    public void startIntake() { // Enables intake
        intakeMotor.set(ControlMode.PercentOutput, Constants.MechanismConstants.intakeSpeed);
        isRunning = true;
    }

    public void stopIntake() { // Disables intake
        intakeMotor.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }

    public void toggleIntake() {
        if (isRunning) stopIntake();
        else startIntake();
    } // Toggles intake

    public void invertIntake() { // Inverts intake wheel direction
        isInverted = !this.isInverted;
        int sign = isInverted ? 1 : -1;
        if (isRunning) intakeMotor.set(ControlMode.PercentOutput, sign * Constants.MechanismConstants.intakeSpeed);
    }

    public void extendPneumatic() { // Extends pneumatic
        rightIntakePneumatic.set(DoubleSolenoid.Value.kForward);
        leftIntakePneumatic.set(DoubleSolenoid.Value.kForward);
        isExtended = true;
    }

    public void retractPneumatic() { // Retracts pneumatic
        rightIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
        leftIntakePneumatic.set(DoubleSolenoid.Value.kReverse);
        isExtended = false;
    }

    public void togglePneumatic() { // Toggles solenoid state
        if (rightIntakePneumatic.get() == DoubleSolenoid.Value.kForward) {
            retractPneumatic();
        } else {
            extendPneumatic();
        }
    }

    public void startIntakeToIndexerMotor() {
        intakeToIndexerMotor.set(Constants.MechanismConstants.intakeToIndexerSpeed);
    }

    public void stopIntakeToIndexerMotor() {
        intakeToIndexerMotor.set(0);
    }

    public void toggleIntakeToIndexerMotor() {
        if (intakeToIndexerRunning) {
            stopIntakeToIndexerMotor();
        } else startIntakeToIndexerMotor();
    }

    public void invertedIntakeToIndexerMotor() {
        intakeToIndexerMotor.set(-intakeToIndexerSpeed);
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void disable() { // Disables intake subsystem
        stopIntake();
        stopIntakeToIndexerMotor();
        retractPneumatic();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Active", isRunning);
    }
}


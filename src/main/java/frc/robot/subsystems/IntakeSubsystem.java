package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.indexer.EnsureResidualBeltCountdownIsNotRunning;
import frc.robot.commands.indexer.RunBeltResidually;
import frc.robot.commands.indexer.StartBelt;

import static frc.robot.Constants.PneumaticConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    // Configure intake motor, solenoid, and booleans
    private final TalonFX intakeMotor = new TalonFX(Constants.MechanismConstants.intakeMotorPort);
    private final DoubleSolenoid rightIntakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortRightA, pneumaticPortRightB);
    private final DoubleSolenoid leftIntakePneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, pneumaticPortLeftA, pneumaticPortLeftB);

    private final IndexerSubsystem indexer;

    public boolean isRunning = false;
    public boolean isExtended = false;
    public boolean isInverted = false;

    public IntakeSubsystem(IndexerSubsystem indexer) {
        intakeMotor.configFactoryDefault(); // Initialize motor set up
        intakeMotor.setNeutralMode(NeutralMode.Coast);
        intakeMotor.configOpenloopRamp(0.1); // Ramp up (Trapezoid)
        intakeMotor.configClosedloopRamp(0.1); // Ramp down (Trapezoid)

        this.indexer = indexer;
    }

    public void startIntake() { // Enables intake
        intakeMotor.set(ControlMode.PercentOutput, Constants.MechanismConstants.intakeSpeed);
        isRunning = true; if (indexer.beltRunning) new EnsureResidualBeltCountdownIsNotRunning(indexer).schedule();
        new StartBelt(indexer).schedule(); //if the belt is already running when we start the intake there's a countdown going that's going to stop it if we don't interrupt it first
    }

    /**
     *
     *
     *
     *
     *
     *
     *
     *
     * I LOVE YOU SO MUCH JACOB
     *
     *
     * UWU
     *
     *
     *
     *
     * - SEBASTIAN
     *
     */

    public void stopIntake() { // Disables intake
        intakeMotor.set(ControlMode.PercentOutput, 0);
        isRunning = false; new RunBeltResidually(indexer).withInterrupt(indexer.getResidualBeltFlag()).schedule();
    }

    public void toggleIntake() { if (isRunning) stopIntake(); else startIntake(); } // Toggles intake

    public void invertIntake() { // Inverts intake wheel direction
        isInverted = !this.isInverted;
        int sign = isInverted ? 1: -1;
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

    public void disable() { // Disables intake subsystem
        stopIntake();
        retractPneumatic();
    }

    @Override
    public void periodic() {SmartDashboard.putBoolean("Intake", isRunning);}
}


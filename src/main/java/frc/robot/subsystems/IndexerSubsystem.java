package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.indexer.EnsureResidualBeltCountdownIsNotRunning;
import frc.robot.lib.colorwheel.ColorWheelUtils;
import frc.robot.lib.colorwheel.WheelColors;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.MechanismConstants.indexerBeltMotorPort;
import static frc.robot.Constants.MechanismConstants.indexerMecanumMotorPort;


public class IndexerSubsystem extends SubsystemBase {
    // Configure motors and booleans
    private final TalonFX indexerMotor = new TalonFX(indexerMecanumMotorPort);
    private final TalonFX beltMotor = new TalonFX(indexerBeltMotorPort);
    private final ColorWheelUtils colorWheelUtils = new ColorWheelUtils();

    public WheelColors currentColor = WheelColors.GREEN;
    public double currentProximity = 2000;

    public boolean indexerRunning = false;
    public boolean beltRunning = false;
    public boolean residualBeltFlag = false;

    public IndexerSubsystem() {
        beltMotor.setInverted(true);
    }

    public void startIndexer() { // Enables indexer
        indexerMotor.set(ControlMode.PercentOutput, Constants.MechanismConstants.indexerSpeed);
        indexerRunning = true;
    }

    public void stopIndexer() { // Disables indexer
        indexerMotor.set(ControlMode.PercentOutput, 0);
        indexerRunning = false;
    }

    public void toggleIndexer() { if (indexerRunning) stopIndexer(); else startIndexer(); } // Toggles indexer

    public void startBelt() { // Enables belt
        beltMotor.set(ControlMode.PercentOutput, Constants.MechanismConstants.beltSpeed);
        beltRunning = true; new EnsureResidualBeltCountdownIsNotRunning(this).schedule();
    }

    public void stopBelt() { // Disables belt
        beltMotor.set(ControlMode.PercentOutput, 0);
        beltRunning = false;
    }

    public void toggleBelt() { if (beltRunning) stopBelt(); else startBelt(); } // Toggles belt

    public boolean ballPrimed() { return currentProximity > 1800; }

    public WheelColors primedBallColor() { return currentColor; }

    public BooleanSupplier getResidualBeltFlag() {return () -> residualBeltFlag;}

    public void disable() { // Disables indexer and belt
        stopIndexer();
        stopBelt();
    }

    @Override
    public void periodic() {
        currentColor = colorWheelUtils.currentColor();
        currentProximity = colorWheelUtils.currentProximity();

        SmartDashboard.putBoolean("Indexer", indexerRunning);
    }
}


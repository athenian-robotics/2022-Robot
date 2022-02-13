package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.colorwheel.ColorWheelUtils;
import frc.robot.lib.colorwheel.WheelColors;

import static frc.robot.Constants.MechanismConstants.indexerMotorPort;


public class IndexerSubsystem extends SubsystemBase {
    // Configure motor and booleans
    private final TalonFX indexerMotor = new TalonFX(indexerMotorPort);
    private final ColorWheelUtils colorWheelUtils = new ColorWheelUtils();

    public WheelColors currentColor = WheelColors.GREEN;
    public double currentProximity = 0;

    public boolean indexerRunning = false;

    public IndexerSubsystem() {
        indexerMotor.setInverted(true);
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

    public boolean ballPrimed() { return currentProximity > 1800; }

    public WheelColors primedBallColor() { return currentColor; }

    public void disable() { // Disables indexer and belt
        stopIndexer();
    }

    @Override
    public void periodic() {
        currentColor = colorWheelUtils.currentColor();
        currentProximity = colorWheelUtils.currentProximity();

        SmartDashboard.putBoolean("Indexer", indexerRunning);
    }
}


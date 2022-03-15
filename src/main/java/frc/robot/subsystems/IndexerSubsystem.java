package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.colorwheel.ColorWheelUtils;
import frc.robot.lib.colorwheel.WheelColors;

import static frc.robot.Constants.MechanismConstants.indexerMotorPort;
import static frc.robot.Constants.MechanismConstants.indexerSpeed;


public class IndexerSubsystem extends SubsystemBase {
    // Configure motor and booleans
    private final TalonFX indexerMotor = new TalonFX(indexerMotorPort);
    private final ColorWheelUtils colorWheelUtils = new ColorWheelUtils();

    private final NetworkTableEntry indexerNTE;

    public WheelColors currentColor = WheelColors.GREEN;
    public double currentProximity = 0;
    public boolean ballIndexed = false;

    public boolean indexerRunning = false;

    public IndexerSubsystem() {
        indexerMotor.setInverted(false);

        indexerNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Indexer Active", false)
                .getEntry();
    }

    public void startIndexer() {
        // Enables indexer
        indexerMotor.set(ControlMode.PercentOutput, indexerSpeed);
        indexerRunning = true;
    }

    public void reverseIndexer() {
        indexerMotor.set(ControlMode.PercentOutput, -indexerSpeed);
        indexerRunning = true;
    }

    public void stopIndexer() { // Disables indexer
        indexerMotor.set(ControlMode.PercentOutput, 0);
        indexerRunning = false;
    }

    public void setIndexer(int power) {
        indexerMotor.set(ControlMode.PercentOutput, power);
    }

    public void toggleIndexer() {
        if (indexerRunning) stopIndexer();
        else startIndexer();
    } // Toggles indexer


    public boolean ballPrimed() {
        return currentProximity > 10;
    }

    public WheelColors primedBallColor() {
        return currentColor;
    }

    public void disable() { // Disables indexer and belt
        stopIndexer();
    }

    @Override
    public void periodic() {
        currentColor = colorWheelUtils.currentColor();
        currentProximity = colorWheelUtils.currentProximity();

        indexerNTE.setBoolean(indexerRunning);
        SmartDashboard.putBoolean("Indexer Active", indexerRunning);
        SmartDashboard.putNumber("Proximity", currentProximity);
        SmartDashboard.putBoolean("Ball Indexed", ballIndexed);
    }
}


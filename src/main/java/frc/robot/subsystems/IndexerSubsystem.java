package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MechanismConstants.indexerBeltMotorPort;
import static frc.robot.Constants.MechanismConstants.indexerMecanumMotorPort;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX indexerMotor = new TalonFX(indexerMecanumMotorPort);
    private final TalonFX beltMotor = new TalonFX(indexerBeltMotorPort);
    public boolean isRunning = false;

    public IndexerSubsystem() {

    }

    public void startIndexer() {
        indexerMotor.set(ControlMode.PercentOutput, 0.7);
        beltMotor.set(ControlMode.PercentOutput, 0.7);
        isRunning = true;
    }

    public void stopIndexer() {
        indexerMotor.set(ControlMode.PercentOutput, 0);
        beltMotor.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }

    public void toggleIndexer() {
        if (isRunning) {
            stopIndexer();
        } else {
            startIndexer();
        }
    }

    public void startBelt() {
        beltMotor.set(ControlMode.PercentOutput, 0.7);
        isRunning = true;
    }

    public void stopBelt() {
        beltMotor.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }

    public void toggleBelt() {
        if (isRunning) {
            stopBelt();
        } else {
            startBelt();
        }
    }

    @Override
    public void periodic() {
    }
}


package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.MechanismConstants.indexerBeltMotorPort;
import static frc.robot.Constants.MechanismConstants.indexerMecanumMotorPort;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX indexerMotor = new TalonFX(indexerMecanumMotorPort);
    private final TalonFX beltMotor = new TalonFX(indexerBeltMotorPort);
    public boolean indexerRunning = false;
    public boolean beltRunning = false;

    public IndexerSubsystem() {

    }

    public void startIndexer() {
        indexerMotor.set(ControlMode.PercentOutput, Constants.MechanismConstants.indexerSpeed);
        indexerRunning = true;
    }

    public void stopIndexer() {
        indexerMotor.set(ControlMode.PercentOutput, 0);
        indexerRunning = false;
    }

    public void toggleIndexer() {
        if (indexerRunning) {
            stopIndexer();
        } else {
            startIndexer();
        }
    }

    public void startBelt() {
        beltMotor.set(ControlMode.PercentOutput, Constants.MechanismConstants.beltSpeed);
        beltRunning = true;
    }

    public void stopBelt() {
        beltMotor.set(ControlMode.PercentOutput, 0);
        beltRunning = false;
    }

    public void toggleBelt() {
        if (beltRunning) {
            stopBelt();
        } else {
            startBelt();
        }
    }


    public void disable() {
        stopIndexer();
        stopBelt();
    }

    @Override
    public void periodic() {
    }
}


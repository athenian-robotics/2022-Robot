package frc.robot.subsystems;

import static frc.robot.Constants.MechanismConstants.indexerMotorPort;
import static frc.robot.Constants.MechanismConstants.indexerSpeed;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX indexerMotor = new TalonFX(indexerMotorPort);

  public IndexerSubsystem() {
    indexerMotor.setInverted(false);
    indexerMotor.enableVoltageCompensation(true);
  }

  public void startIndexer() {
    indexerMotor.set(ControlMode.PercentOutput, indexerSpeed);
  }

  public void startIndexerReversed() {
    indexerMotor.set(ControlMode.PercentOutput, -indexerSpeed);
  }

  public void stopIndexer() {
    indexerMotor.set(ControlMode.PercentOutput, 0);
  }

  public void disable() { // Disables indexer and belt
    stopIndexer();
  }

  public void periodic() {}
}

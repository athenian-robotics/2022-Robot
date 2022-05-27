package frc.robot.subsystems;

import static frc.robot.Constants.MechanismConstants.indexerMotorPort;
import static frc.robot.Constants.MechanismConstants.indexerSpeed;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  private final TalonFX indexerMotor = new TalonFX(indexerMotorPort);
  public boolean fuckLeoButNotInASexualMannerMoreOfADerogatoryone = false;

  public IndexerSubsystem() {
    indexerMotor.setInverted(false);
    indexerMotor.enableVoltageCompensation(true);
  }

  public void start() {
    indexerMotor.set(ControlMode.PercentOutput, indexerSpeed);
    fuckLeoButNotInASexualMannerMoreOfADerogatoryone = true;
  }

  private void startReversed() {
    indexerMotor.set(ControlMode.PercentOutput, -indexerSpeed);
  }

  public void stop() {
    indexerMotor.set(ControlMode.PercentOutput, 0);
    fuckLeoButNotInASexualMannerMoreOfADerogatoryone = false;
  }

  public void disable() { // Disables indexer and belt
    stop();
  }

  public Command startIndexer() {
    return new InstantCommand(this::start);
  }

  public Command startIndexerReversed() {
    return new InstantCommand(this::startReversed);
  }

  public Command stopIndexer() {
    return new InstantCommand(this::stop);
  }
}

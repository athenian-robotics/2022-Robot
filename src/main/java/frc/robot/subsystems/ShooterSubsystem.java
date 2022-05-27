package frc.robot.subsystems;

import static com.ctre.phoenix.motorcontrol.NeutralMode.Coast;
import static frc.robot.Constants.MechanismConstants.shooterMotorPortA;
import static frc.robot.Constants.MechanismConstants.shooterMotorPortB;
import static frc.robot.RobotContainer.limelight;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.lib.controllers.SimpleVelocitySystem;
import frc.robot.lib.motors.TalonFXFactory;
import frc.robot.lib.shooterData.ShooterDataTable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
  // Setup motors, pid controller, and booleans
  private final WPI_TalonFX shooterMotorFront;
  private final SimpleVelocitySystem sys;
  private final ShooterDataTable shooterDataTable;
  @Log.ToString public ShooterState state = ShooterState.IDLE;
  private final PoseEstimator poseEstimator;
  private final NetworkTableEntry shooterPowerNTE;

  public ShooterSubsystem(PoseEstimator poseEstimator, ShooterDataTable shooterDataTable) {
    this.poseEstimator = poseEstimator;
    this.shooterDataTable = shooterDataTable;
    shooterMotorFront = TalonFXFactory.createDefaultTalon(shooterMotorPortA);
    shooterMotorFront.setInverted(false);
    WPI_TalonFX shooterMotorBack =
        TalonFXFactory.createPermanentSlaveTalon(shooterMotorPortB, shooterMotorPortA);
    shooterMotorBack.setInverted(false);

    shooterMotorFront.setNeutralMode(Coast);
    shooterMotorBack.setNeutralMode(Coast);
    shooterMotorBack.follow(shooterMotorFront);

    // for shooterDataTable values
    shooterPowerNTE =
        Shuffleboard.getTab("852 - Dashboard")
            .add("Shooter Power", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", 0, "max", 500))
            .getEntry();

    shooterMotorFront.configVoltageCompSaturation(12);
    shooterMotorBack.configVoltageCompSaturation(12);
    shooterMotorFront.enableVoltageCompensation(true);
    shooterMotorBack.enableVoltageCompensation(true);

    sys =
        new SimpleVelocitySystem(
            Constants.Shooter.ks,
            Constants.Shooter.kv,
            Constants.Shooter.ka,
            Constants.Shooter.maxError,
            Constants.Shooter.maxControlEffort,
            Constants.Shooter.modelDeviation,
            Constants.Shooter.encoderDeviation,
            Constants.looptime);
  }

  private void setShooterPower(double power) { // Enables both wheels
    if (power > 1.0) power = 1.0;
    if (power < 0.0) power = 0.0;
    shooterMotorFront.set(ControlMode.PercentOutput, power);
  }

  @Log
  private double getWheelSpeed() {
    return shooterMotorFront.getSelectedSensorVelocity() / 2048;
  }

  private boolean atSetpoint() {
    return Math.abs(sys.getError()) < Constants.Shooter.maxError;
  }

  @Log
  @SuppressWarnings("unused")
  private double getRPS() {
    return sys.getVelocity();
  }

  public ShooterState getState() {
    return state;
  }

  @Override
  public void periodic() {
    if (state != ShooterState.IDLE) {
      sys.update(getWheelSpeed());
      if (state == ShooterState.TESTING) {
        sys.set(shooterPowerNTE.getDouble(0));
      } else {
        sys.set(shooterDataTable.getSpecs(limelight.getDistance()).getPower());
      }
      setShooterPower(sys.getOutput());
      if (atSetpoint()) {
        state = ShooterState.READY;
      }
    } else {
      setShooterPower(0);
      // shooter data table values
    }
  }

  public Command requestShot() {
    return new InstantCommand(() -> state = ShooterState.APPROACHING);
  }

  public Command discard() {
    return new InstantCommand(() -> state = ShooterState.DISCARD);
  }

  public Command idle() {
    return new InstantCommand(() -> state = ShooterState.IDLE);
  }

  public Command waitUntilReady() {
    return new WaitUntilCommand(() -> state == ShooterState.READY);
  }

  public enum ShooterState {
    IDLE, // default state
    DISCARD, // discarding enemy balls
    READY, // at setpoint and withing tolerance
    APPROACHING, // approaching setpoint
    TESTING // for collecting shooter data table values
  }

  public Command test() {
    return new InstantCommand(() -> state = ShooterState.TESTING);
  }
}

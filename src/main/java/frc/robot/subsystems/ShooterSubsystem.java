package frc.robot.subsystems;

import static com.ctre.phoenix.motorcontrol.NeutralMode.Coast;
import static frc.robot.Constants.MechanismConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.controllers.SimpleVelocitySystem;
import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
  // Setup motors, pid controller, and booleans
  private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
  private final NetworkTableEntry shooterAdjustmentNTE;

  public boolean shooterRunning = false;
  public double shuffleboardShooterPower;
  public double shuffleboardShooterAdjustment;

  private final SimpleVelocitySystem sys;
  private double shooterRPS = 0;

  public ShooterSubsystem() {
    shooterMotorFront.setInverted(false);
    TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
    shooterMotorBack.setInverted(false);

    shooterMotorFront.setNeutralMode(Coast);
    shooterMotorBack.setNeutralMode(Coast);
    shooterMotorBack.follow(shooterMotorFront);

    shooterAdjustmentNTE =
        Shuffleboard.getTab("852 - Dashboard")
            .add("Shooter Power Adjustment", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.75, "max", 1.25, "default value", 1))
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

  public void setShooterPower(double power) { // Enables both wheels
    if (power > 1.0) power = 1.0;
    if (power < 0.0) power = 0.0;
    shooterMotorFront.set(ControlMode.PercentOutput, power);
    shooterRunning = true;
  }

  public double getWheelSpeed() {
    return shooterMotorFront.getSelectedSensorVelocity() / 4096;
  }

  public void setRPS(double rps) {
    sys.set(rps * shuffleboardShooterAdjustment);
    shooterRunning = true;
    shooterRPS = rps * shuffleboardShooterAdjustment;
  }

  public void disable() { // Disables shooter
    setShooterPower(0);
    shooterRPS = 0;
    shooterRunning = false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Shooter Active", shooterRunning);
    SmartDashboard.putNumber("Shooter Speed", getWheelSpeed());
    SmartDashboard.putNumber("Shooter Power", shooterRPS);

    shuffleboardShooterAdjustment = shooterAdjustmentNTE.getDouble(1);

    if (shooterRunning) {
      sys.update(getWheelSpeed());
      setShooterPower(sys.getOutput());
    }
  }
}

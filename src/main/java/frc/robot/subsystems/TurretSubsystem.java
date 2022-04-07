package frc.robot.subsystems;

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;
import static frc.robot.Constants.MechanismConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;

public class TurretSubsystem extends SubsystemBase {
  public final WPI_TalonFX turretMotor = new WPI_TalonFX(turretMotorPort);
  public final SimpleMotorFeedforward feed;
  public final ProfiledPIDController turretPID;

  private final LimelightSubsystem limelight;
  private final LimelightDataLatch turretToleranceDistanceLatch;

  public boolean PIDRunning = false;
  public boolean bangBangRunning = false;
  public double currentTurretToleranceRadians = Math.toRadians(1);

  public TurretSubsystem(LimelightSubsystem limelight) {
    this.limelight = limelight;
    turretMotor.setInverted(false);
    turretMotor.setNeutralMode(NeutralMode.Brake);

    turretToleranceDistanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 16);
    feed =
        new SimpleMotorFeedforward(Constants.Turret.ks, Constants.Turret.kv, Constants.Turret.ka);
    turretPID =
        new ProfiledPIDController(
            3.47, 0, 0.05, new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI / 4));

    setTurretStartingAngleDegrees(
        -180); // assume default position is turret starting facing backwards counterclockwise
  }

  public void turnTurret(double power) {
    if (power > 0 && getTurretAngleRadians() < maximumTurretAngleRadians
        || power < 0 && getTurretAngleRadians() > minimumTurretAngleRadians) {
      turretMotor.set(
          ControlMode.PercentOutput,
          power > turretTurnSpeed ? turretTurnSpeed : Math.max(power, -turretTurnSpeed));
    } else if (power == 0.0) {
      disable();
    } else {
      turretMotor.set(0);
    }
  }

  public void turnTurretWithVoltage(double voltage) {
    if (voltage == 0.0) {
      disable();
    } else {
      turretMotor.setVoltage(voltage);
    }
  }

  // Primarily for use in auto routines where we need to know where the shooter starts
  public void setTurretStartingAngleDegrees(double position) {
    turretMotor.setSelectedSensorPosition(2048 * position / 36);
    turretPID.setGoal(Math.toRadians(position));
  }

  // CW Positive
  public void setTurretSetpointRadians(double angle) {
    if (angle > maximumTurretAngleRadians)
      angle = maximumTurretAngleRadians - angle + minimumTurretAngleRadians;
    if (angle < minimumTurretAngleRadians)
      angle = minimumTurretAngleRadians - angle + maximumTurretAngleRadians;
    turretPID.setGoal(angle);
    PIDRunning = true;
  }

  public double getTurretAngleRadians() {
    return Math.toRadians(turretMotor.getSelectedSensorPosition() * 36 / 2048);
  }

  private void updateCurrentTurretTolerance() {
    try {
      if (turretToleranceDistanceLatch.unlocked()) {
        currentTurretToleranceRadians = Math.toRadians(5) / turretToleranceDistanceLatch.open();
        SmartDashboard.putNumber("turret tolerance", Math.toDegrees(currentTurretToleranceRadians));
      }
    } catch (GoalNotFoundException e) {
      limelight.addLatch(turretToleranceDistanceLatch.reset());
    }
  }

  public void disable() {
    turretMotor.set(PercentOutput, 0);
    PIDRunning = false;
    bangBangRunning = false;
  }

  @Override
  public void periodic() {
    updateCurrentTurretTolerance();
    if (PIDRunning) {
      double PIDOutput = turretPID.calculate(getTurretAngleRadians());
      if (Math.abs(turretPID.getPositionError()) > currentTurretToleranceRadians) {
        turnTurret(PIDOutput);
      } else turnTurret(0);
    }

    //    if (bangBangRunning) {
    //      double bangBangOffset = bangBangSetpointRadians - getTurretAngleRadians();
    //      if (Math.abs(bangBangOffset) >= currentShooterToleranceRadians) {
    //        turnTurret(
    //            Math.signum(bangBangOffset) * slowTurretTurnSpeed + Math.min(turretTurnSpeed /
    // bangBangOffset, turretTurnSpeed));
    //      } else {
    //        turretMotor.set(0);
    //      }
    //    }
  }
}

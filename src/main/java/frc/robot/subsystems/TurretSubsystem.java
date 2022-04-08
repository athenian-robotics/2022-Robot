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
  private final DrivetrainSubsystem drivetrain;

  public double currentTurretToleranceRadians = Math.toRadians(1);
  public boolean PIDRunning = false;

  public TurretSubsystem(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    turretMotor.setInverted(false);
    turretMotor.setNeutralMode(NeutralMode.Brake);

    turretToleranceDistanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 16);
    feed =
        new SimpleMotorFeedforward(Constants.Turret.ks, Constants.Turret.kv, Constants.Turret.ka);
    turretPID =
        new ProfiledPIDController(
            5.85, 0.02, 1.188, new TrapezoidProfile.Constraints(4 * Math.PI, 2 * Math.PI));

    // 3.85, 0.02, 0.068 new trap(pi/2,pi/4)

    setTurretStartingAngleDegrees(
        -180); // assume default position is turret starting facing backwards counterclockwise
    setTurretSetpointRadians(getTurretAngleRadians());
  }

  public void turnTurret(double power) {
    turretMotor.set(
        ControlMode.PercentOutput,
        power > turretTurnSpeed ? turretTurnSpeed : Math.max(power, -turretTurnSpeed));
  }

  // Primarily for use in auto routines where we need to know where the shooter starts
  public void setTurretStartingAngleDegrees(double position) {
    turretMotor.setSelectedSensorPosition(2048 * position / 36);
    turretPID.setGoal(Math.toRadians(position));
  }

  // CW Positive
  public void setTurretSetpointRadians(double angle) {
    turretPID.setGoal(angle);
    PIDRunning = true;
  }

  public double getTurretAngleRadians() {
    return Math.toRadians(turretMotor.getSelectedSensorPosition() * 36 / 2048);
  }

  private void updateCurrentTurretTolerance() {
    try {
      if (turretToleranceDistanceLatch.unlocked()) {
        currentTurretToleranceRadians = Math.toRadians(6) / turretToleranceDistanceLatch.open();
        SmartDashboard.putNumber("turret tolerance", Math.toDegrees(currentTurretToleranceRadians));
      }
    } catch (GoalNotFoundException e) {
      limelight.addLatch(turretToleranceDistanceLatch.reset());
    }
  }

  public void disable() {
    turretMotor.set(PercentOutput, 0);
    setTurretSetpointRadians(getTurretAngleRadians());
    PIDRunning = false;
  }

  @Override
  public void periodic() {
    updateCurrentTurretTolerance();
    if (PIDRunning) {
      double PIDOutput = turretPID.calculate(getTurretAngleRadians());
      if (Math.abs(turretPID.getPositionError())
          > currentTurretToleranceRadians
              * 0.9 /* fixes discrepancy in PID tolerance and limelight tolerance */) {
        turnTurret(PIDOutput);
      } else turnTurret(0);
    }
  }
}

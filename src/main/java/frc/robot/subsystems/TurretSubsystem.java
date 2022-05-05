package frc.robot.subsystems;

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;
import static frc.robot.Constants.MechanismConstants.turretMotorPort;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class TurretSubsystem extends SubsystemBase implements Loggable {
  public final WPI_TalonFX turretMotor = new WPI_TalonFX(turretMotorPort);

  private final LinearSystemLoop<N2, N1, N1> turretLoop;
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI / 4);
  private TrapezoidProfile.State lastReference = new TrapezoidProfile.State();
  TrapezoidProfile.State goal = new TrapezoidProfile.State();

  public boolean LQRRunning = true;

  @Log private double turretGoal;
  @Log private double nextTurretVolts;
  @Log private double positionSetpoint;

  public TurretSubsystem() {
    // CCW +
    turretMotor.setInverted(true);
    turretMotor.setNeutralMode(NeutralMode.Brake);

    LinearSystem<N2, N1, N1> turretPlant =
        LinearSystemId.identifyPositionSystem(Constants.Turret.kv, Constants.Turret.ka);
    KalmanFilter<N2, N1, N1> kalmanFilter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            turretPlant,
            VecBuilder.fill(0.001, 0.25288),
            VecBuilder.fill(0.01),
            0.020);
    LinearQuadraticRegulator<N2, N1, N1> turretController =
        new LinearQuadraticRegulator<>(
            turretPlant,
            VecBuilder.fill(Units.degreesToRadians(1), Units.degreesToRadians(4)),
            VecBuilder.fill(12),
            0.020);
    turretController.latencyCompensate(turretPlant, 0.020, 0.017);
    turretLoop = new LinearSystemLoop<>(turretPlant, turretController, kalmanFilter, 3, 0.020);

    // 3.85, 0.02, 0.068 new trap(pi/2,pi/4)
    setTurretStartingAngleDegrees(
       180); // assume default position is turret starting facing backwards counterclockwise
    setTurretSetpointRadians(getTurretAngleRadians());
    turretLoop.reset(VecBuilder.fill(getTurretAngleRadians(), getTurretSpeed()));
  }

  public void turnTurret(double power) {
    turretMotor.set(
        ControlMode.PercentOutput,
        power > turretTurnSpeed ? turretTurnSpeed : Math.max(power, -turretTurnSpeed));
  }

  // Primarily for use in auto routines where we need to know where the shooter starts
  public void setTurretStartingAngleDegrees(double angle) {
    turretMotor.setSelectedSensorPosition(2048 * angle / 36);
    goal = new TrapezoidProfile.State(angle, 0);
  }

  // CCW Positive, 0-360
  public void setTurretSetpointRadians(double angle) {
    goal = new TrapezoidProfile.State(angle, 0);
    LQRRunning = true;
  }

  @Log
  public double getTurretAngleRadians() {
    return Math.toRadians(turretMotor.getSelectedSensorPosition() * 36 / 2048);
  }

  @Log
  public double getTurretSpeed() {
    return Math.toRadians(turretMotor.getSelectedSensorVelocity() * 36 / 2048);
  }

  public void disable() {
    turretMotor.set(PercentOutput, 0);
    setTurretSetpointRadians(getTurretAngleRadians());
    LQRRunning = false;
  }

  public boolean atGoal(double tolerance) {
    return Math.abs(getTurretAngleRadians() - turretGoal) < tolerance;
  }

  @Override
  public void periodic() {
    turretGoal = goal.position /* + drivetrain.getPositionOffset(goal.position, distance) */;
    if (turretGoal > 270) turretGoal -= 360;
    if (LQRRunning) {
      lastReference =
          (new TrapezoidProfile(
                  constraints, new TrapezoidProfile.State(turretGoal, 0), lastReference))
              .calculate(0.02);
      positionSetpoint = lastReference.position;
      turretLoop.setNextR(positionSetpoint, 0);
      turretLoop.correct(VecBuilder.fill(getTurretAngleRadians()));
      turretLoop.predict(0.02);
      nextTurretVolts = turretLoop.getU(0);
      turretMotor.setVoltage(nextTurretVolts + Constants.Turret.ks * Math.signum(turretLoop.getXHat(0)));
    } else turnTurret(0);
  }
}

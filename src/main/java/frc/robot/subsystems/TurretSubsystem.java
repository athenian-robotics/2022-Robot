package frc.robot.subsystems;

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;
import static frc.robot.Constants.MechanismConstants.turretMotorPort;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;
import static frc.robot.RobotContainer.drivetrain;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;

public class TurretSubsystem extends SubsystemBase {
  public final WPI_TalonFX turretMotor = new WPI_TalonFX(turretMotorPort);

  private final LimelightSubsystem limelight;
  private final LimelightDataLatch turretToleranceDistanceLatch;
  private final LinearSystemLoop<N2, N1, N1> turretLoop;
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI / 4);
  private TrapezoidProfile.State lastReference = new TrapezoidProfile.State();
  TrapezoidProfile.State goal = new TrapezoidProfile.State();

  public double currentTurretToleranceRadians = Math.toRadians(1);
  public boolean LQRRunning = false;
  private double distance;

  public TurretSubsystem(LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
    this.limelight = limelight;
    turretMotor.setInverted(false);
    turretMotor.setNeutralMode(NeutralMode.Brake);

    turretToleranceDistanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 16);
    LinearSystem<N2, N1, N1> turretPlant =
        LinearSystemId.identifyPositionSystem(Constants.Turret.kv, Constants.Turret.ka);
    KalmanFilter<N2, N1, N1> kalmanFilter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            turretPlant,
            VecBuilder.fill(0.47283, 5.6516),
            VecBuilder.fill(0.01),
            0.020);
    LinearQuadraticRegulator<N2, N1, N1> turretController =
        new LinearQuadraticRegulator<>(
            turretPlant,
            VecBuilder.fill(Units.degreesToRadians(1), Units.degreesToRadians(4)),
            VecBuilder.fill(12),
            0.020);
    turretController.latencyCompensate(turretPlant, 0.020, 0.017);
    turretLoop = new LinearSystemLoop<>(turretPlant, turretController, kalmanFilter, 12, 0.020);

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
  public void setTurretStartingAngleDegrees(double angle) {
    turretMotor.setSelectedSensorPosition(2048 * angle / 36);
    goal = new TrapezoidProfile.State(angle, 0);
  }

  // CW Positive
  public void setTurretSetpointRadians(double angle) {
    goal = new TrapezoidProfile.State(angle, 0);
    LQRRunning = true;
  }

  public double getTurretAngleRadians() {
    return Math.toRadians(turretMotor.getSelectedSensorPosition() * 36 / 2048);
  }

  private void updateCurrentTurretTolerance() {
    try {
      if (turretToleranceDistanceLatch.unlocked()) {
        distance = turretToleranceDistanceLatch.open();
        currentTurretToleranceRadians = Math.toRadians(6) / distance;
        SmartDashboard.putNumber("turret tolerance", Math.toDegrees(currentTurretToleranceRadians));
      }
    } catch (GoalNotFoundException e) {
      limelight.addLatch(turretToleranceDistanceLatch.reset());
    }
  }

  public void disable() {
    turretMotor.set(PercentOutput, 0);
    setTurretSetpointRadians(getTurretAngleRadians());
    LQRRunning = false;
  }

  @Override
  public void periodic() {
    double Tgoal =
        Math.toDegrees(
            goal.position
                + drivetrain.getPositionOffset(goal.position + getTurretAngleRadians(), distance));
    SmartDashboard.putNumber("goal", Tgoal);
    SmartDashboard.putNumber("curr", Math.toDegrees(getTurretAngleRadians()));
    updateCurrentTurretTolerance();
    if (LQRRunning) {
      lastReference =
          (new TrapezoidProfile(
                  constraints, new TrapezoidProfile.State(Tgoal, goal.velocity), lastReference))
              .calculate(0.02);
      turretLoop.setNextR(lastReference.position, lastReference.velocity);
      turretLoop.correct(VecBuilder.fill(getTurretAngleRadians()));
      turretLoop.predict(0.02);
      double next = turretLoop.getU(0);
      turretMotor.setVoltage(next);
    } else turnTurret(0);
  }
}

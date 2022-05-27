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
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class TurretSubsystem extends SubsystemBase implements Loggable {
  public final WPI_TalonFX turretMotor = new WPI_TalonFX(turretMotorPort);

  private final LinearSystemLoop<N2, N1, N1> turretLoop;
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI / 4);
  private TrapezoidProfile.State lastReference = new TrapezoidProfile.State();
  @Log private double turretGoalOffset;
  @Log private double nextTurretPercent;
  @Log private double positionGoal;
  private final LimelightSubsystem limelight;
  private final PoseEstimator poseEstimator;
  @Log.ToString private State state = State.IDLE;
  private LinearFilter iir = LinearFilter.singlePoleIIR(0.01, 0.02);

  public enum State {
    IDLE,
    LQR,
  }

  public TurretSubsystem(LimelightSubsystem limelight, PoseEstimator poseEstimator) {
    this.limelight = limelight;
    this.poseEstimator = poseEstimator;
    // CCW +
    turretMotor.setInverted(true);
    turretMotor.setNeutralMode(NeutralMode.Coast);
    LinearSystem<N2, N1, N1> turretPlant =
        LinearSystemId.identifyPositionSystem(Constants.Turret.kv, Constants.Turret.ka);
    KalmanFilter<N2, N1, N1> kalmanFilter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            turretPlant,
            VecBuilder.fill(0.1, 0.48521),
            VecBuilder.fill(0.00060),
            0.020);
    LinearQuadraticRegulator<N2, N1, N1> turretController =
        new LinearQuadraticRegulator<>(
            turretPlant,
            VecBuilder.fill(Units.degreesToRadians(.9), Units.degreesToRadians(30)),
            VecBuilder.fill(5),
            0.020);
    turretController.latencyCompensate(turretPlant, 0.020, 0.01);
    turretLoop = new LinearSystemLoop<>(turretPlant, turretController, kalmanFilter, 5, 0.020);
    turretMotor.setSelectedSensorPosition(2048 * Math.toRadians(180) / 36);
    turretLoop.reset(VecBuilder.fill(getTurretAngleRadians(), getTurretSpeed()));
  }

  @Log
  private double getTurretAngleRadians() {
    return turretMotor.getSelectedSensorPosition() / 2048 * Math.PI * 2 / 10;
  }

  public void turnTurret(double power) {
    turretMotor.set(
        ControlMode.PercentOutput,
        power > turretTurnSpeed ? turretTurnSpeed : Math.max(power, -turretTurnSpeed));
  }

  @Log
  public double getTurretSpeed() {
    return Math.toRadians(turretMotor.getSelectedSensorVelocity() / 2048 * Math.PI * 2 / 10);
  }

  public void disable() {
    turretMotor.set(PercentOutput, 0);
  }

  public boolean atGoal(double tolerance) {
    return Math.abs(getTurretAngleRadians() - turretGoalOffset) < tolerance;
  }

  private Transform2d relativeLimelightPosition() {
    return new Transform2d(
        new Translation2d(0.1524, new Rotation2d(getTurretAngleRadians())),
        new Rotation2d(getTurretAngleRadians()));
  }

  @Override
  public void periodic() {
    //    Pose2d pose =
    //        PhotonUtils.estimateFieldToRobot(
    //            limelight.getTrackedTarget().getCameraToTarget(),
    //            Constants.hub,
    //            relativeLimelightPosition());
    //    poseEstimator.addVisionPose(pose, limelight.getTimeSinceLastUpdate());
    if (state == State.LQR) {
      //      if (limelight.getTimeSinceLastUpdate() > 0.5) {
      //        turretGoal =
      //            Math.atan(
      //                poseEstimator.getPose().minus(Constants.hub).getTranslation().getY()
      //                    / poseEstimator.getPose().minus(Constants.hub).getTranslation().getX());
      //      } else {
      turretGoalOffset =
          -Math.toRadians(iir.calculate(limelight.getTrackedTarget().getYaw()))
              + getTurretAngleRadians() /* + poseEstimator.get*/;
      lastReference =
          (new TrapezoidProfile(
                  constraints, new TrapezoidProfile.State(turretGoalOffset, 0), lastReference))
              .calculate(0.02);
      positionGoal = lastReference.position;
      turretLoop.setNextR(positionGoal, lastReference.velocity);
      turretLoop.correct(VecBuilder.fill(getTurretAngleRadians()));
      turretLoop.predict(0.02);
      nextTurretPercent =
          (turretLoop.getU(0)) - Constants.Turret.ks * Math.signum(turretLoop.getNextR(0));
      turretMotor.setVoltage((nextTurretPercent));
    } else if (state == State.IDLE) {
      turretMotor.set(0);
    }
  }

  public Command waitUntilSetpoint() {
    return new WaitUntilCommand(() -> atGoal(0.1));
  }
}

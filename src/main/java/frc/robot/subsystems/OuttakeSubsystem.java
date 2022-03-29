package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.controllers.SimpleVelocitySystem;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;

import java.util.Map;

import static com.ctre.phoenix.motorcontrol.NeutralMode.Coast;
import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;
import static frc.robot.Constants.MechanismConstants.*;
import static frc.robot.Constants.looptime;

public class OuttakeSubsystem extends SubsystemBase {
    public final WPI_TalonFX turretMotor = new WPI_TalonFX(turretMotorPort);
    public final SimpleMotorFeedforward feed;

    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
    private final Servo leftHoodAngleServo = new Servo(2);
    private final Servo rightHoodAngleServo = new Servo(3);
    private final NetworkTableEntry shooterAdjustmentNTE;
    private final LimelightDataLatch distanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 5);
    private final LimelightSubsystem limelightSubsystem;
    private final SimpleVelocitySystem sys;
    public boolean shooterRunning = false;
    public boolean turretRunning = false;
    public boolean lqrRunning = false;
    public double shuffleboardShooterPower;
    public double shuffleboardShooterAdjustment;
    public double currentShooterToleranceDegrees = 1;
    private double shooterRPS = 0;
    private double setpointRadians;
    private final LinearSystemLoop<N2, N1, N1> turretLoop;
    public OuttakeSubsystem(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;

        shooterMotorFront.setInverted(false);
        TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
        shooterMotorBack.setInverted(false);
        turretMotor.setInverted(false);

        turretMotor.setNeutralMode(NeutralMode.Brake);
        shooterMotorFront.setNeutralMode(Coast);
        shooterMotorBack.setNeutralMode(Coast);
        shooterMotorBack.follow(shooterMotorFront);

        leftHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos
        rightHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos

        shooterAdjustmentNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Shooter Power Adjustment", 1)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.75, "max", 1.25, "default value", 1))
                .getEntry();

        shooterMotorFront.configVoltageCompSaturation(12);
        shooterMotorBack.configVoltageCompSaturation(12);
        shooterMotorFront.enableVoltageCompensation(true);
        shooterMotorBack.enableVoltageCompensation(true);

        sys = new SimpleVelocitySystem(Constants.Shooter.ks, Constants.Shooter.kv, Constants.Shooter.ka,
                Constants.Shooter.maxError, Constants.Shooter.maxControlEffort,
                Constants.Shooter.modelDeviation, Constants.Shooter.encoderDeviation,
                looptime);

        this.feed = new SimpleMotorFeedforward(Constants.Turret.ks, Constants.Turret.kv,
                Constants.Turret.ka);

        setTurretStartingAngleDegrees(-180); //assume default position is turret starting facing backwards
        // counterclockwise
        LinearSystem<N2, N1, N1> turretPlant = LinearSystemId.identifyPositionSystem(Constants.Turret.kv,
                Constants.Turret.ka);

        // model is probs 1 deg off (check sys id later)
        // also 1 deg/s is the second param
        // 1 enc tick is this many
        KalmanFilter<N2, N1, N1> offsetFilter = new KalmanFilter<>(
                Nat.N2(),
                Nat.N1(),
                turretPlant,
                VecBuilder.fill(Math.toRadians(1), Math.toRadians(1)),// model is probs 1 deg off (check sys id later)
                // also 1 deg/s is the second param
                VecBuilder.fill(Math.toRadians(0.0176)), // 1 enc tick is this many
                looptime
        );

        // 1 deg/s error and 1 deg error
        // control effort
        LinearQuadraticRegulator<N2, N1, N1> turretController = new LinearQuadraticRegulator<>(
                turretPlant,
                VecBuilder.fill(Math.toRadians(1), Math.toRadians(1)), // 1 deg/s error and 1 deg error
                VecBuilder.fill(12), // control effort
                looptime
        );


        turretLoop =
                new LinearSystemLoop<>(turretPlant, turretController, offsetFilter, 12, looptime);


    }

    public void setShooterPower(double power) { // Enables both wheels
        setShooterFront(power);
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

    public void setShooterFront(double power) {
        if (power > 1.0) power = 1.0;
        if (power < 0.0) power = 0.0;
        shooterMotorFront.set(ControlMode.PercentOutput, power);
    }

    public void turnTurret(double power) {
        if (power == 0.0) {
            stopTurret();
        } else {
            turretMotor.set(ControlMode.PercentOutput, power > turretTurnSpeed ? turretTurnSpeed : Math.max(power,
                    -turretTurnSpeed));
        }
    }

    public double getHoodAngle() { //Takes the average of the angles (0-1) and scales it into a degree measurement
        return ((maximumHoodAngle - minimumHoodAngle) * (leftHoodAngleServo.getAngle() + rightHoodAngleServo.getAngle()) / 360) + minimumHoodAngle;
    }

    public void setHoodAngle(double angle) {
        if (angle >= minimumHoodAngle && angle <= maximumHoodAngle) {
            leftHoodAngleServo.setAngle(180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle)); //
            // 0 - 180 DEGREES
            rightHoodAngleServo.setAngle(180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle));
            // 0 - 180 DEGREES
        }
    }

    public void stopShooter() { // Disables shooter
        setShooterPower(0);
        shooterRPS = 0;
        shooterRunning = false;
    }

    public void stopHood() {
        leftHoodAngleServo.setSpeed(0);
        rightHoodAngleServo.setSpeed(0);
    }

    public void stopTurret() {
        turretMotor.set(PercentOutput, 0);
        turretRunning = false;
        lqrRunning = false;
    }

    public void setTurretStartingAngleDegrees(double position) {
        //Primarily for use in auto routines where we need to know where the shooter starts
        turretMotor.setSelectedSensorPosition(2048 * position / 36);
    }

    //CW Positive
    public void setTurretPositionRadians(double angle) {
        setpointRadians = angle;
        lqrRunning = true;
    }

    public double getTurretAngleRadians() {
        return Math.toRadians(turretMotor.getSelectedSensorPosition() * 36 / 2048);
    }

    public void disable() {
        stopShooter();
        stopHood();
        stopTurret();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake Active", shooterRunning);
        SmartDashboard.putNumber("Shooter Speed", getWheelSpeed());
        SmartDashboard.putNumber("Hood Angle", getHoodAngle());
        SmartDashboard.putNumber("Turret Angle", getTurretAngleRadians());
        SmartDashboard.putNumber("Shooter Power", shooterRPS);

        shuffleboardShooterAdjustment = shooterAdjustmentNTE.getDouble(1);

        if (shooterRunning) {
            sys.update(getWheelSpeed());
            setShooterPower(sys.getOutput());
        }

        if (turretRunning) {
            turretLoop.setNextR(VecBuilder.fill(setpointRadians, 0));
            turretLoop.correct(VecBuilder.fill(getTurretAngleRadians()));
            turretLoop.predict(looptime);
            double next = turretLoop.getU(0);
            turretMotor.setVoltage(next);
        } else {
            turretLoop.setNextR(VecBuilder.fill(getTurretAngleRadians(), 0));
        }



        try {
            if (distanceLatch.unlocked()) {
                currentShooterToleranceDegrees = turretShootZoneRadians / distanceLatch.open();
            }
        } catch (GoalNotFoundException e) {
            limelightSubsystem.addLatch(distanceLatch.reset());
        }
    }


}
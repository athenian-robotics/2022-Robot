package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;
import static frc.robot.Constants.MechanismConstants.*;

public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
    private final WPI_TalonFX turretMotor = new WPI_TalonFX(turretMotorPort);
    private final Servo leftHoodAngleServo = new Servo(2);
    private final Servo rightHoodAngleServo = new Servo(3);

    public final PIDController turretPID = new PIDController(.0556, 0.003, 0.009);

    private final NetworkTableEntry shooterAdjustmentNTE;
    private final LimelightDataLatch distanceLatch = new LimelightDataLatch(LimelightDataType.DISTANCE, 5);
    private final LimelightSubsystem limelightSubsystem;

    public boolean shooterRunning = false;
    public boolean turretRunning = false;
    public double shuffleboardShooterPower;
    public double shuffleboardShooterAdjustment;
    public double currentShooterToleranceDegrees = 1;

    private final SimpleVelocitySystem sys;
    private double shooterRPS = 0;
    private double angle;
    private SlewRateLimiter filter = new SlewRateLimiter(Math.PI/2);

    public OuttakeSubsystem(LimelightSubsystem limelightSubsystem) {
        this.limelightSubsystem = limelightSubsystem;

        shooterMotorFront.setInverted(false);
        TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
        shooterMotorBack.setInverted(false);
        turretMotor.setInverted(false);

        turretMotor.setNeutralMode(NeutralMode.Brake);
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorBack.setNeutralMode(NeutralMode.Coast);
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
                Constants.looptime);

        setTurretStartingAngle(-180); //assume default position is turret starting facing backwards counterclockwise
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
        if (getTurretAngle() < maximumTurretAngle && power > 0 || getTurretAngle() > minimumTurretAngle && power < 0) {
            turretMotor.set(ControlMode.PercentOutput, power > turretTurnSpeed ? turretTurnSpeed : Math.max(power, -turretTurnSpeed));
        } else if (power == 0.0) {
            stopTurret();
        } else turretMotor.set(ControlMode.PercentOutput, 0);
    }

    public void turnTurretWithVoltage(double voltage) {
        System.out.println(voltage);
        if (getTurretAngle() < maximumTurretAngle && voltage > 0 || getTurretAngle() > minimumTurretAngle && voltage < 0) {
            turretMotor.setVoltage(MathUtil.clamp(voltage, -2, 2));
        } else if (voltage == 0.0) {
            stopTurret();
        } else turretMotor.setVoltage(0.0);
    }

    public void setHoodAngle(double angle) {
        if (angle >= minimumHoodAngle && angle <= maximumHoodAngle) {
            leftHoodAngleServo.setAngle(180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle)); //
            // 0 - 180 DEGREES
            rightHoodAngleServo.setAngle(180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle));
            // 0 - 180 DEGREES
        }
    }

    public double getHoodAngle() { //Takes the average of the angles (0-1) and scales it into a degree measurement
        return ((maximumHoodAngle - minimumHoodAngle) * (leftHoodAngleServo.getAngle() + rightHoodAngleServo.getAngle()) / 360) + minimumHoodAngle;
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
    }

    public void setTurretStartingAngle(double position) {
        //Primarily for use in auto routines where we need to know where the shooter starts
        turretMotor.setSelectedSensorPosition(2048 * position / 36);
    }

    public double getTurretAngle() {
        return Math.toRadians(turretMotor.getSelectedSensorPosition() * 36 / 2048);
    }

    public void setTurretPosition(double angle){
        turretPID.setSetpoint(angle);
        this.angle = angle;
        turretRunning = true;
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
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
        SmartDashboard.putNumber("Shooter Power", shooterRPS);

        shuffleboardShooterAdjustment = shooterAdjustmentNTE.getDouble(1);

        if (shooterRunning) {
            sys.update(getWheelSpeed());
            setShooterPower(sys.getOutput());
        }

        if (turretRunning) {
            turnTurretWithVoltage(turretPID.calculate(getTurretAngle())); // + feed.calc ?

            try {
                if (distanceLatch.unlocked()) {
                    currentShooterToleranceDegrees = 3/distanceLatch.open();
                }
            } catch (GoalNotFoundException e) {
                limelightSubsystem.addLatch(distanceLatch.reset());
            }

        }
    }
}
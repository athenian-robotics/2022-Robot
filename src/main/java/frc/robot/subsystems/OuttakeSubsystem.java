package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.controllers.SimpleVelocitySystem;

import java.util.Map;

import static frc.robot.Constants.MechanismConstants.*;

public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
    private final TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
    private final TalonFX turretMotor = new TalonFX(turretMotorPort);

    private final Servo leftHoodAngleServo = new Servo(2);
    private final Servo rightHoodAngleServo = new Servo(3);

    private final NetworkTableEntry shooterNTE;
    private final NetworkTableEntry turretAngleNTE;
    private final NetworkTableEntry shooterAdjustmentNTE;
    private final NetworkTableEntry shooterActiveNTE;
    public final PIDController turretAnglePID;

    public boolean shooterRunning = false;
    public boolean turretActive = false;
    public double shuffleboardShooterPower = 0;
    public double shuffleboardShooterAdjustment = 0;

    public double shuffleBoardTurretAngle = 0;
    private final SimpleVelocitySystem sys;

    public OuttakeSubsystem() {
        shooterMotorFront.setInverted(false);
        shooterMotorBack.setInverted(false);
        turretMotor.setInverted(false);

        turretAnglePID = new PIDController(0.012, 0.005, 0.0015);
        turretAnglePID.setSetpoint(0); //Always trying to minimize our offset
        turretAnglePID.setTolerance(0.5);


        turretMotor.setNeutralMode(NeutralMode.Coast);
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorBack.setNeutralMode(NeutralMode.Coast);
        shooterMotorBack.follow(shooterMotorFront);

        leftHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos
        rightHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos

        shooterAdjustmentNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Shooter Power Adjustment", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.8, "max", 1.2, "default value", 1))
                .getEntry();

        shooterActiveNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Shooter Active", false)
                .getEntry();

        //TODO remove! for testing only
        shooterNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Shooter Power", shuffleboardShooterPower)
                .withWidget(BuiltInWidgets.kTextView)
                .withProperties(Map.of())
                .getEntry();

        turretAngleNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Turret Angle", 8)
                .withWidget(BuiltInWidgets.kTextView)
                .withProperties(Map.of("min", 8, "max", 41))
                .getEntry();

        shooterMotorFront.configVoltageCompSaturation(12);
        shooterMotorBack.configVoltageCompSaturation(12);
        shooterMotorFront.enableVoltageCompensation(true);
        shooterMotorBack.enableVoltageCompensation(true);
        sys = new SimpleVelocitySystem(Constants.Shooter.ks, Constants.Shooter.kv, Constants.Shooter.ka,
                Constants.Shooter.maxError, Constants.Shooter.maxControlEffort,
                Constants.Shooter.modelDeviation, Constants.Shooter.encoderDeviation,
                Constants.looptime);

        setTurretPosition(-180); //assume default position is turret starting counterclockwise backwards
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
    }

    public void setShooterFront(double power) {
        if (power > 1.0 || power < 0.0) return;
        shooterMotorFront.set(ControlMode.PercentOutput, power);
    }

    public void turnTurret(double power) {
        if (getTurretAngle() < maximumTurretAngle && power > 0 || getTurretAngle() > minimumTurretAngle && power < 0) {
            turretMotor.set(ControlMode.PercentOutput, power > turretTurnSpeed ? turretTurnSpeed : power < -turretTurnSpeed ? -turretTurnSpeed : power);
        } else turretMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setHoodAngle(double angle) {
        if (angle >= minimumHoodAngle && angle <= maximumHoodAngle) {
            leftHoodAngleServo.setAngle(180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle)); // 0 - 180 DEGREES
            rightHoodAngleServo.setAngle(180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle)); // 0 - 180 DEGREES
        }
    }

    public double getHoodAngle() { //Takes the average of the angles (0-1) and scales it into a degree measurement
        return ((maximumHoodAngle - minimumHoodAngle) * (leftHoodAngleServo.getAngle() + rightHoodAngleServo.getAngle()) / 360) + minimumHoodAngle;
    }

    public void stopShooter() { // Disables shooter
        setRPS(0);
        shooterRunning = false;
    }

    public void stopHood() {
        leftHoodAngleServo.setSpeed(0);
        rightHoodAngleServo.setSpeed(0);
    }

    public void stopTurret() {
        turretMotor.set(TalonFXControlMode.PercentOutput, 0);
        turretAnglePID.reset();
    }

    public double getFrontShooterAcceleration() {
        return shooterMotorFront.getErrorDerivative();
    }

    public double getBackShooterAcceleration() {
        return shooterMotorBack.getErrorDerivative();
    }

    public void setTurretPosition(double position) {
        //Primarily for use in auto routines where we need to know where the shooter starts
        turretMotor.setSelectedSensorPosition(2048 * position / 36);
    }

    public double getTurretAngle() {
        return turretMotor.getSelectedSensorPosition() * 36 / 2048;
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
        SmartDashboard.putNumber("HoodAngle", leftHoodAngleServo.get());
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
        SmartDashboard.putNumber("Shooter RPS", sys.getVelocity());

        shooterActiveNTE.setBoolean(shooterRunning);
        shuffleboardShooterPower = shooterNTE.getDouble(1);
        shuffleBoardTurretAngle = turretAngleNTE.getDouble(8);
        shuffleboardShooterAdjustment = shooterAdjustmentNTE.getDouble(1);

        if (shooterRunning) {
            sys.update(getWheelSpeed());
            setShooterPower(sys.getOutput());
        }
    }
}
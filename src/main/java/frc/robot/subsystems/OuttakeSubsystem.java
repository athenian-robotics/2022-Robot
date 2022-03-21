package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.controllers.SimplePositionSystem;
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


    private final NetworkTableEntry shooterAdjustmentNTE;
    public final PIDController turretAnglePID;
    public final PIDController limelightTurretAnglePID;

    public boolean shooterRunning = false;
    public double shuffleboardShooterPower;
    public double shuffleboardShooterAdjustment;

    private final SimpleVelocitySystem sys;
    private final SimplePositionSystem syst;
    private double shooterRPS = 0;

    public OuttakeSubsystem() {
        shooterMotorFront.setInverted(false);
        shooterMotorBack.setInverted(false);
        turretMotor.setInverted(false);

        turretMotor.setNeutralMode(NeutralMode.Coast);
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorBack.setNeutralMode(NeutralMode.Coast);
        shooterMotorBack.follow(shooterMotorFront);

        leftHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos
        rightHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos

        shooterAdjustmentNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Shooter Power Adjustment", 1)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.8, "max", 1.2, "default value", 1))
                .getEntry();

        turretAnglePID = new PIDController(0.007, 0.001, 0.001);
        turretAnglePID.setSetpoint(0); //Always trying to minimize our offset
        turretAnglePID.setTolerance(0.5);

        limelightTurretAnglePID = new PIDController(0.007, 0.001, 0.0012);
//        //(0.006, 0.00085, 0.004);
        limelightTurretAnglePID.setSetpoint(0);
        limelightTurretAnglePID.setTolerance(1);

        shooterMotorFront.configVoltageCompSaturation(12);
        shooterMotorBack.configVoltageCompSaturation(12);
        shooterMotorFront.enableVoltageCompensation(true);
        shooterMotorBack.enableVoltageCompensation(true);
        sys = new SimpleVelocitySystem(Constants.Shooter.ks, Constants.Shooter.kv, Constants.Shooter.ka,
                Constants.Shooter.maxError, Constants.Shooter.maxControlEffort,
                Constants.Shooter.modelDeviation, Constants.Shooter.encoderDeviation,
                Constants.looptime);

        setTurretAngle(0); //assume default position is turret starting counterclockwise backwards
        syst = new SimplePositionSystem(Constants.Turret.ks, Constants.Turret.kv, Constants.Turret.ka, Constants.Turret.maxError, Constants.Turret.maxControlEffort, Constants.Turret.modelDeviation, Constants.Turret.encoderDeviation, Constants.looptime);
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
        shooterRPS = rps*shuffleboardShooterAdjustment;
    }

    public void setShooterFront(double power) {
        if (power > 1.0) power = 1.0;
        if (power < 0.0) power = 0.0;
        shooterMotorFront.set(ControlMode.PercentOutput, power);
    }

    public void turnTurret(double power) {
        if (getTurretAngle() < maximumTurretAngle && power > 0 || getTurretAngle() > minimumTurretAngle && power < 0) {
            turretMotor.set(ControlMode.PercentOutput, power > turretTurnSpeed ? turretTurnSpeed : Math.max(power, -turretTurnSpeed));
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
        setShooterPower(0);
        shooterRPS = 0;
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

    public void setTurretAngle(double position) {
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
        SmartDashboard.putNumber("Hood Angle", getHoodAngle());
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
        SmartDashboard.putNumber("Shooter Power", shooterRPS);

        shuffleboardShooterAdjustment = shooterAdjustmentNTE.getDouble(1);

        if (shooterRunning) {
            sys.update(getWheelSpeed());
            setShooterPower(sys.getOutput());
        }
    }
}
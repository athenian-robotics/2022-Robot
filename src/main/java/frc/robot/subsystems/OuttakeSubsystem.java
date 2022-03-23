package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.controllers.SimplePositionSystem;
import frc.robot.lib.controllers.SimpleVelocitySystem;

import java.util.Map;

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;
import static frc.robot.Constants.MechanismConstants.*;

public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
    private final TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
    private final TalonFX turretMotor = new TalonFX(turretMotorPort);
    private final Servo leftHoodAngleServo = new Servo(2);
    private final Servo rightHoodAngleServo = new Servo(3);


    private final NetworkTableEntry shooterAdjustmentNTE;

    private final PIDController turretPID;

    public boolean shooterRunning = false;
    public boolean turretRunning = false;
    public double shuffleboardShooterPower;
    public double shuffleboardShooterAdjustment;

    private final SimpleVelocitySystem sys;
    private final SimplePositionSystem sysT;
    private double shooterRPS = 0;

    public OuttakeSubsystem() {
        shooterMotorFront.setInverted(false);
        shooterMotorBack.setInverted(false);
        turretMotor.setInverted(false);

        turretMotor.setNeutralMode(NeutralMode.Brake);
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorBack.setNeutralMode(NeutralMode.Coast);
        shooterMotorBack.follow(shooterMotorFront);

        turretPID = new PIDController(0.007,0.001,0.001);
        turretPID.setTolerance(0.5);

        turretMotor.configClosedloopRamp(0.5);
        turretMotor.config_kP(0, 0.02);

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

        sysT = new SimplePositionSystem(Constants.Turret.ks, Constants.Turret.kv, Constants.Turret.ka,
                Constants.Turret.maxError, Constants.Turret.maxControlEffort, Constants.Turret.modelDeviation,
                Constants.Turret.encoderDeviation, Constants.looptime);

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
        return turretMotor.getSelectedSensorPosition() * 36 / 2048;
    }

    public void setTurretPosition(double angle){
        // system
        turretMotor.set(ControlMode.Position, 2048 * angle / 36);
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
    }
}
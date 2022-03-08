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
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.controllers.SimpleVelocitySystem;
import frc.robot.lib.shooterData.ShooterDataTable;

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
    public final PIDController turretAnglePID;

    private final ShooterDataTable table =  new ShooterDataTable();
    public boolean shooterRunning = false;
    public boolean turretActive = false;
    public double shuffleboardShooterPower = 0;

    public double shuffleBoardTurretAngle = 0;
    private final SimpleVelocitySystem sys;


    public OuttakeSubsystem() {
        shooterMotorFront.setInverted(false);
        shooterMotorBack.setInverted(false);
        turretMotor.setInverted(false);

        turretAnglePID = new PIDController(0.005, 0.0005, 0.0005);
        turretAnglePID.setSetpoint(0); //Always trying to minimize our offset
        turretAnglePID.setTolerance(0.5);


        turretMotor.setNeutralMode(NeutralMode.Brake);
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorBack.setNeutralMode(NeutralMode.Coast);
        shooterMotorBack.follow(shooterMotorFront);

        leftHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos
        rightHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos

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

        setTurretPosition(180);
    }

    public void setShooterPower(double power) { // Enables both wheels
        setShooterFront(power);
        shooterRunning = true;
    }

    public double getWheelSpeed() {
        return shooterMotorFront.getSelectedSensorVelocity() / 4096;
    }

    public void setRPS(double rps) {
        sys.set(rps);
        sys.update(getWheelSpeed());
        setShooterPower(sys.getOutput());
    }

    public void setShooterFront(double power) {
        if (power > 1.0 || power < 0.0) return;
        shooterMotorFront.set(ControlMode.PercentOutput, power);
    }

    public void manualAdjustHoodAngle(double theta) {
        leftHoodAngleServo.setAngle(leftHoodAngleServo.getAngle() + theta);
        rightHoodAngleServo.setAngle(rightHoodAngleServo.getAngle() + theta);
    }

    public void turnTurret(double power) {
        if (getTurretPosition() < maximumTurretAngle && power > 0 || getTurretPosition() > minimumTurretAngle && power < 0) {
            turretMotor.set(ControlMode.PercentOutput, power);
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
        setShooterFront(0);
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

    public void setTurretActive(boolean active) {
        turretActive = active;
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

    public double getTurretPosition() {
        return turretMotor.getSelectedSensorPosition() * 36 / 2048;
    }

    public void disable() {
        stopShooter();
        stopHood();
        stopTurret();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake", shooterRunning);
        SmartDashboard.putNumber("Shooter Speed", getWheelSpeed());
        SmartDashboard.putNumber("HoodAngle", leftHoodAngleServo.get());
        SmartDashboard.putNumber("Turret Angle", getTurretPosition());
        SmartDashboard.putNumber("Shooter RPS", sys.getVelocity());
        shuffleboardShooterPower = shooterNTE.getDouble(1);
        shuffleBoardTurretAngle = turretAngleNTE.getDouble(8);
        setHoodAngle(shuffleBoardTurretAngle);

        // HOOD ANGLE LINEAR SERVOS
        if (FightStick.fightStickJoystick.getY() < -0.5) { // Inverted
            this.manualAdjustHoodAngle(1);
        } else if (FightStick.fightStickJoystick.getY() > 0.5) {
            this.manualAdjustHoodAngle(-1);
        }
    }
}


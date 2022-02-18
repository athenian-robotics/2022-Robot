package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.GoalNotFoundException;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.shooter.ShooterDataTable;

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
    private final PIDController turretAnglePID;
    private final LimelightSubsystem limelight;

    private ShooterDataTable table =  new ShooterDataTable();
    public boolean shooterRunning = false;
    public boolean turretActive = false;
    private double shuffleboardShooterPower = 0;


    public OuttakeSubsystem(LimelightSubsystem ll) {
        limelight = ll;

        shooterMotorFront.setInverted(false);
        shooterMotorBack.setInverted(false);

        turretAnglePID = new PIDController(0, 0, 0);
        turretAnglePID.setSetpoint(0); //Always trying to minimize our offset

        leftHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos
        rightHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0); //Manufacturer specified for Actuonix linear servos

        shooterNTE = Shuffleboard.getTab("852 - Dashboard")
                .add("Shooter Power", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 100))
                .getEntry();
    }

    public void setShooterPower(double power) { // Enables both wheels
        setShooterFront(power);
        setShooterBack(power);
        shooterRunning = true;
    }

    public void setShooterFront(double power) {
        if (power > 1.0 || power < 0.0) return;
        shooterMotorFront.set(ControlMode.PercentOutput, power);
    }

    public void setShooterBack(double power) {
        if (power > 1.0 || power < 0.0) return;
        shooterMotorBack.set(ControlMode.PercentOutput, power);
    }

    public void manualAdjustHoodAngle(double theta) {
        leftHoodAngleServo.setAngle(leftHoodAngleServo.getAngle() + theta);
        rightHoodAngleServo.setAngle(rightHoodAngleServo.getAngle() + theta);
    }

    public void manualAdjustTurret(double power) {
        //turretMotor.set(ControlMode.PercentOutput, power);}
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
        setShooterBack(0);
        shooterRunning = false;
    }

    public void stopHood() {
        leftHoodAngleServo.setSpeed(0);
        rightHoodAngleServo.setSpeed(0);
    }

    public void stopTurret() {
        turretMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void setTurretActive(boolean active) {
        turretActive = active;
    }

    public double getFrontShooterAcceleration() {
        return shooterMotorFront.getErrorDerivative();}

    public double getBackShooterAcceleration() {
        return shooterMotorBack.getErrorDerivative();}

    public void disable() {
        stopShooter();
        stopHood();
        stopTurret();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake", shooterRunning);
        shuffleboardShooterPower = shooterNTE.getDouble(0);

        if (turretActive) { //Sets turret with limelight to PID to aim at the center of the goal
            try {
                turretMotor.set(ControlMode.PercentOutput, -limelight.getLimelightOutputAtIndex(1));
            } catch (GoalNotFoundException e) {/* SEARCH FOR GOAL */}
        } else { //Checks Fight Stick X Axis for Moving the Turret
            if (FightStick.fightStickJoystick.getX() < 0) {
                manualAdjustTurret(-idleTurretSpeed);
            } else if (FightStick.fightStickJoystick.getX() > 0) {
                manualAdjustTurret(idleTurretSpeed);
            } else {
                manualAdjustTurret(0);
            }
        }
    }
}


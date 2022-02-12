package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.controllers.FightStick;

import java.util.Map;

import static frc.robot.Constants.MechanismConstants.*;


public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
    private final TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
    private final Servo leftHoodAngleServo = new Servo(2);
    private final Servo rightHoodAngleServo = new Servo(3);
    //ColorWheelUtils colorWheel = new ColorWheelUtils();
    PIDController frontShooterPID;
    PIDController backShooterPID;
    NetworkTableEntry shooterNTE;

    public boolean shooterRunning = false;
    private double currentFrontShooterPower = 0.0;
    private double currentBackShooterPower = 0.0;

    public OuttakeSubsystem() {
        shooterMotorFront.setInverted(false);
        shooterMotorBack.setInverted(false);

        frontShooterPID = new PIDController(0, 0 ,0);
        backShooterPID = new PIDController(0, 0 ,0);

        leftHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        rightHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

        Shuffleboard.getTab("852-Dashboard")
                .add("Shooter Power", 1).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 100));
    }

    public void setShooterPower(double power) { // Enables both wheels
        setShooterFront(power);
        setShooterBack(power);
        shooterRunning = true;
    }

    public void setShooterFront(double power) {
        if ( power > 1.0 || power < 0.0) return;
        currentFrontShooterPower = power;
        shooterMotorFront.set(ControlMode.PercentOutput, currentFrontShooterPower);
    }

    public void setShooterBack(double power) {
        if (power>1.0 || power<0.0) return;
        currentBackShooterPower = power;
        shooterMotorBack.set(ControlMode.PercentOutput, currentBackShooterPower);
    }

    public void manualAdjustHoodAngle(double theta) {
        leftHoodAngleServo.setAngle(leftHoodAngleServo.getAngle() + theta);
        rightHoodAngleServo.setAngle(rightHoodAngleServo.getAngle() + theta);
    }
    /*
    public void setHoodAngle(double angle) {
        if (angle >= minimumHoodAngle && angle <= maximumHoodAngle) {
            leftHoodAngleServo.setAngle(180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle)); // 0 - 180 DEGREES
            rightHoodAngleServo.setAngle(180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle)); // 0 - 180 DEGREES
        }
    }
    */

    public double getHoodAngle() { //DEGREES + DEFAULT 9
        return ((maximumHoodAngle - minimumHoodAngle) * (leftHoodAngleServo.getPosition() + rightHoodAngleServo.getPosition()) / 360) + minimumHoodAngle; }

    public void stopShooter() { // Disables shooter
        setShooterFront(0);
        setShooterBack(0);
        shooterRunning = false;
    }

    public void stopHood() { leftHoodAngleServo.setSpeed(0); rightHoodAngleServo.setSpeed(0); }

    public void disable() {
        stopShooter();
        stopHood();
        //stopTurret();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake", shooterRunning);
    }
}


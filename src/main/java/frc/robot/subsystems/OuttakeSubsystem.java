package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.colorwheel.ColorWheelUtils;
import frc.robot.lib.controllers.FightStick;

import static frc.robot.Constants.MechanismConstants.*;


public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
    private final TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
    private final Servo leftHoodAngleServo = new Servo(1);
    private final Servo rightHoodAngleServo = new Servo(2);
    ColorWheelUtils colorWheel = new ColorWheelUtils();
    PIDController frontShooterPID;
    PIDController backShooterPID;

    public boolean shooterRunning = false;
    private double currentFrontShooterPower = 0.0;
    private double currentBackShooterPower = 0.0;
    private double currentHoodAngle;


    public OuttakeSubsystem() {
        shooterMotorFront.setInverted(true);
        shooterMotorBack.setInverted(false);

        frontShooterPID = new PIDController(0, 0 ,0);
        backShooterPID = new PIDController(0, 0 ,0);

        //setHoodAngle(45);
        currentHoodAngle = getHoodAngle();
        leftHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        rightHoodAngleServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    }

    public void setShooterPower(double power) { // Enables both wheels
        setShooterFront(power);
        setShooterBack(power);
        shooterRunning = true;
    }

    public void setShooterFront(double power) { if (power>1.0 || power<0.0) return; currentFrontShooterPower = power; }

    public void setShooterBack(double power) { if (power>1.0 || power<0.0) return; currentBackShooterPower = power; }

    public void setHoodAngle(double angle) { if (angle>=minimumHoodAngle && angle<=maximumHoodAngle) { leftHoodAngleServo.setAngle(180*(angle-minimumHoodAngle)/(maximumHoodAngle-minimumHoodAngle)); rightHoodAngleServo.setAngle(180*(angle-minimumHoodAngle)/(maximumHoodAngle-minimumHoodAngle)); }}

    public double getHoodAngle() { return ((maximumHoodAngle-minimumHoodAngle)*(leftHoodAngleServo.getPosition() + rightHoodAngleServo.getPosition())/360) + minimumHoodAngle; } //DEGREES + DEFAULT 9

    public double getTargetedHoodAngle() { return currentHoodAngle; }

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
        colorWheel.updateColorsOnDashboard();
        colorWheel.currentColor();
        SmartDashboard.putBoolean("Outtake", shooterRunning);

        if (FightStick.fightStickJoystick.getY() < 0) {
            leftHoodAngleServo.setAngle(leftHoodAngleServo.getAngle() + 1);
            rightHoodAngleServo.setAngle(rightHoodAngleServo.getAngle() + 1);
        } else if (FightStick.fightStickJoystick.getY() > 0) {
            leftHoodAngleServo.setAngle(leftHoodAngleServo.getAngle() - 1);
            rightHoodAngleServo.setAngle(rightHoodAngleServo.getAngle() - 1);
        }
    }
}


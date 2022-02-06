package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.outtake.AdjustHoodAngle;
import frc.robot.lib.LinearServo;
import frc.robot.lib.controllers.FightStick;

import static frc.robot.Constants.MechanismConstants.*;


public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
    private final TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
    private final LinearServo leftHoodAngleServo = new LinearServo(1, 140, 24);
    private final LinearServo rightHoodAngleServo = new LinearServo(2, 140, 24);

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

        setHoodAngle(45);
        currentHoodAngle = getHoodAngle();
    }

    public void setShooterPower(double power) { // Enables both wheels
        setShooterFront(power);
        setShooterBack(power);
        shooterRunning = true;
    }
    
    public void setShooterFront(double power) { if (power>1.0 || power<0.0) return; currentFrontShooterPower = power; }

    public void setShooterBack(double power) { if (power>1.0 || power<0.0) return; currentBackShooterPower = power; }

    public void setHoodAngle(double angle) { if (angle<=45.0 && angle>=9.0) { leftHoodAngleServo.setPosition(140*(angle-8)/36.0); rightHoodAngleServo.setPosition(140*(angle-8)/36.0); }}

    public double getHoodAngle() { return (36*(leftHoodAngleServo.getPosition() + rightHoodAngleServo.getPosition())/280) + 8; } //DEGREES + DEFAULT 9

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
        //stopHood();
        //stopTurret();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Outtake", shooterRunning);
        leftHoodAngleServo.updateCurPos();
        rightHoodAngleServo.updateCurPos();

        if (FightStick.fightStickJoystick.getThrottle() > 0) new AdjustHoodAngle(this, this.getHoodAngle() + 1);
        if (FightStick.fightStickJoystick.getThrottle() < 0) new AdjustHoodAngle(this, this.getHoodAngle() - 1);
    }
}


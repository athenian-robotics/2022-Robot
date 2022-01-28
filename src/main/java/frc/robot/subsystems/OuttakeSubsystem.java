package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MechanismConstants.*;


public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
    private final TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
    PIDController pid;

    public boolean shooterRunning = false;


    public OuttakeSubsystem() {
        shooterMotorFront.setInverted(true);
        shooterMotorBack.setInverted(false);
        pid = new PIDController(0, 0 ,0);
    }

    private void setShooterPower(double power) { // Enables both wheels
        setShooterFront(power);
        setShooterBack(power);
        shooterRunning = true;
    }
    
    public void setShooterFront(double power) { // Enables front wheels
        shooterMotorFront.set(ControlMode.PercentOutput, power);
        shooterRunning = true;
    }

    public void setShooterBack(double power) { // Enables back wheels
        shooterMotorBack.set(ControlMode.PercentOutput, power);
        shooterRunning = true;
    }

    public void stopShooter() { // Disables shooter
        setShooterPower(0);
        shooterRunning = false;
    }

    public void setHoodAngle(double angle) { pid.setSetpoint(angle); }

    public void disable() {
        stopShooter();
        //stopHood();
        //stopTurret();
    }

    @Override
    public void periodic() {
        /*
        if (hoodUp) {
            double power = pid.calculate(getHoodAngle());
            hoodAngleMotor.set(TalonFXControlMode.PercentOutput, power);
        }
         **/
    }
}


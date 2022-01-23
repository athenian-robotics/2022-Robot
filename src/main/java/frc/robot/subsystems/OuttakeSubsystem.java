package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.EncoderConstants.hoodAngleEncoderPortA;
import static frc.robot.Constants.EncoderConstants.hoodAngleEncoderPortB;
import static frc.robot.Constants.MechanismConstants.*;

public class OuttakeSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor1 = new TalonFX(shooterMotorPortA);
    private final TalonFX shooterMotor2 = new TalonFX(shooterMotorPortB);
    private final TalonFX hoodAngleMotor = new TalonFX(hoodAngleMotorPort); //DEGREES
    private final TalonFX turretMotor = new TalonFX(turretMotorPort);
    private final Encoder hoodEncoder = new Encoder(hoodAngleEncoderPortA, hoodAngleEncoderPortB);

    PIDController pid;

    public boolean shooterRunning = false;
    public boolean hoodUp = false;

    public OuttakeSubsystem() {
        hoodEncoder.setDistancePerPulse(360.0/2048.0);
        hoodEncoder.reset();

        pid = new PIDController(0, 0 ,0);
    }

    public void setShooterVelocity(double velocity) {
        setShooterPower(velocity);
    } //TODO \figure out conversion

    private void setShooterPower(double power) {
        shooterMotor1.set(ControlMode.PercentOutput, power);
        shooterMotor2.set(ControlMode.PercentOutput, power);
        shooterRunning = true;
    }

    public void stopShooter() {
        shooterMotor1.set(ControlMode.PercentOutput, 0);
        shooterMotor2.set(ControlMode.PercentOutput, 0);
        shooterRunning = false;
    }

    public void setHoodAngle(double angle) { pid.setSetpoint(angle); hoodUp = true; }

    public void stopHood() { hoodAngleMotor.set(ControlMode.PercentOutput, 0); hoodUp = false; }

    //positive = spin right
    public void spinTurret(double power) { this.turretMotor.set(ControlMode.PercentOutput, power); }

    public void stopTurret() { turretMotor.set(TalonFXControlMode.PercentOutput, 0); }

    //Degrees off the vertical
    public double getHoodAngle() { return hoodEncoder.getDistance(); }


    public void disable() {
        stopShooter();
        stopHood();
        stopTurret();
    }

    @Override
    public void periodic() {
        if (hoodUp) {
            double power = pid.calculate(getHoodAngle());
            hoodAngleMotor.set(TalonFXControlMode.PercentOutput, power);
        }
    }
}


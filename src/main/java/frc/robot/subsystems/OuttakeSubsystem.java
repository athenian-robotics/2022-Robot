package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.EncoderConstants.hoodAngleEncoderPortA;
import static frc.robot.Constants.EncoderConstants.hoodAngleEncoderPortB;
import static frc.robot.Constants.MechanismConstants.*;

public class OuttakeSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor1 = new TalonFX(shooterMotorPortA);
    private final TalonFX shooterMotor2 = new TalonFX(shooterMotorPortB);
    private final TalonFX hoodAngleMotor = new TalonFX(hoodAngleMotorPort);
    private final TalonFX turretMotor = new TalonFX(turretMotorPort);
    private final Encoder hoodEncoder = new Encoder(hoodAngleEncoderPortA, hoodAngleEncoderPortB);
    private boolean shooterIsRunning = false;

    public OuttakeSubsystem() {
    }

    public void startShooter() {
        shooterMotor1.set(ControlMode.PercentOutput, 1);
        shooterMotor2.set(ControlMode.PercentOutput, 1);
        shooterIsRunning = true;
    }

    public void stopShooter() {
        shooterMotor1.set(ControlMode.PercentOutput, 0);
        shooterMotor2.set(ControlMode.PercentOutput, 0);
        shooterIsRunning = false;

    }

    public void toggleShooter() {
        if (shooterIsRunning) {
            stopShooter();
        } else {
            startShooter();
        }
    }

    public void setShooterPower(double power) {
        shooterMotor1.set(ControlMode.PercentOutput, power);
        shooterMotor2.set(ControlMode.PercentOutput, power);
        shooterIsRunning = true;
    }

    public void setShooterVelocity(double velocity) {
        setShooterPower(velocity);
    }

    public void moveHoodMotor(double power) {
        this.hoodAngleMotor.set(ControlMode.PercentOutput, power);
    }

    //positive = spin right
    public void spinTurret(double power) {
        this.turretMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void periodic() {

    }
}


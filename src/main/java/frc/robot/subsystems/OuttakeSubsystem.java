package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.EncoderConstants.HoodAngleEncoderPort1;
import static frc.robot.Constants.EncoderConstants.HoodAngleEncoderPort2;
import static frc.robot.Constants.MechanismConstants.*;

public class OuttakeSubsystem extends SubsystemBase {

    private final TalonFX shooterMotor1 = new TalonFX(ShooterMotorPort1);
    private final TalonFX shooterMotor2 = new TalonFX(ShooterMotorPort2);
    private final TalonFX hoodAngleMotor = new TalonFX(HoodAngleMotorPort);
    private final TalonFX turretMotor = new TalonFX(TurretMotorPort);
    private final Encoder hoodEncoder = new Encoder(HoodAngleEncoderPort1, HoodAngleEncoderPort2);
    private double defaultHoodAngle = 30;
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


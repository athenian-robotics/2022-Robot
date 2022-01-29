package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MechanismConstants.*;


public class OuttakeSubsystem extends SubsystemBase {
    // Setup motors, pid controller, and booleans
    private final TalonFX shooterMotorFront = new TalonFX(shooterMotorPortA);
    private final TalonFX shooterMotorBack = new TalonFX(shooterMotorPortB);
    private final CANSparkMax hoodAngleMotor = new CANSparkMax(hoodAngleMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final DutyCycleEncoder hoodAngleEncoder = new DutyCycleEncoder(hoodAngleEncoderPort);

    PIDController hoodAnglePID;
    PIDController frontShooterPID;
    PIDController backShooterPID;

    public boolean shooterRunning = false;
    private double currentFrontShooterPower = 0.0;
    private double currentBackShooterPower = 0.0;
    private double currentHoodAngle = defaultHoodAngle;


    public OuttakeSubsystem() {
        hoodAngleEncoder.reset();
        hoodAngleEncoder.setDistancePerRotation(2*Math.PI);

        shooterMotorFront.setInverted(true);
        shooterMotorBack.setInverted(false);

        hoodAnglePID = new PIDController(0, 0 ,0);
        frontShooterPID = new PIDController(0, 0 ,0);
        backShooterPID = new PIDController(0, 0 ,0);

        hoodAnglePID.setSetpoint(defaultHoodAngle);
    }

    private void setShooterPower(double power) { // Enables both wheels
        setShooterFront(power);
        setShooterBack(power);
        shooterRunning = true;
    }
    
    public void setShooterFront(double power) { // Enables front wheels
        if (power>1.0 || power<0.0) return; currentFrontShooterPower = power;
    }

    public void setShooterBack(double power) { // Enables back wheels
        shooterMotorBack.set(ControlMode.PercentOutput, power);
        shooterRunning = true;
    }

    public void stopShooter() { // Disables shooter
        setShooterPower(0);
        shooterRunning = false;
    }

    public void setHoodAngle(double angle) { if (angle>45.0 || angle<10.0) return; this.currentHoodAngle = angle; hoodAnglePID.setSetpoint(angle); }

    public double getHoodAngle() { return hoodAngleEncoder.getDistance(); } //RADIANS

    public void disable() {
        stopShooter();
        //stopHood();
        //stopTurret();
    }

    @Override
    public void periodic() {
        if (shooterRunning) {
            double power = hoodAnglePID.calculate(getHoodAngle());
            hoodAngleMotor.set(power);
        }
    }
}


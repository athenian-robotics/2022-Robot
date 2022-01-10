package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    ChassisSpeeds chassisSpeeds;
    DifferentialDriveOdometry odometry;
    // Creates kinematics object: track width of 27 inches (track width = distance between two sets of wheels)
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27));
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    private final TalonFX[] driveMotors = {
            new TalonFX(Constants.leftFrontDrivePort),
            new TalonFX(Constants.leftRearDrivePort),
            new TalonFX(Constants.rightFrontDrivePort),
            new TalonFX(Constants.rightRearDrivePort)
    };

    public Drivetrain() {
        configureDriveMotors(driveMotors); // Initialize motors
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading())); // Initialize odometry configuration
    }

    /**
     * This method utilizes both joystick outputs on an xbox controller to arcade drive a west-coast drivetrain
     * @param throttle Velocity of drive on the Y axis (Forward & Backwards)
     * @param rot Velocity of rotation
     */
    public void arcadeDrive(double throttle, double rot) {
        double leftPWM = throttle + rot;
        double rightPWM = throttle - rot;

        // Linearize
        double magnitude = Math.max(Math.abs(leftPWM), Math.abs(rightPWM));
        if (magnitude > 1.0) {
            leftPWM /= magnitude;
            rightPWM /= magnitude;
        }
        setMotorPercentOutput(-leftPWM, rightPWM); // Set motor values based off throttle and rotation inputs
    }

    /**
     * Set the front wheels to a desired output. Units: Percentage
     * @param leftOutput Left front wheel output percentage
     * @param rightOutput Right front wheel output percentage
     */
    public void setMotorPercentOutput(double leftOutput, double rightOutput) {
        driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
        driveMotors[1].set(ControlMode.PercentOutput, leftOutput);
        driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
        driveMotors[3].set(ControlMode.PercentOutput, rightOutput);
    }

    /**
     * Factory reset drive motors on initialization
     * @param driveMotors An array of Falcon 500 motors
     */
    public void configureDriveMotors(TalonFX[] driveMotors) {
        for (TalonFX motor: driveMotors) {
            motor.configFactoryDefault(); // Initialize motor set up
            motor.configOpenloopRamp(0.2); // Ramp up (Trapezoid)
            motor.configClosedloopRamp(0.2); // Ramp down (Trapezoid)
            motor.setNeutralMode(NeutralMode.Coast); // Default robot mode should be Coasting
            motor.configForwardSoftLimitEnable(false);
            motor.configReverseSoftLimitEnable(false);

            motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        }
    }

    /**
     * Reset the robot's odometry
     * @param pose Robot's pose as a Pose2d object
     * @param rot Robot's rotation as a Rotation2d object
     */
    public void resetOdometry(Pose2d pose, Rotation2d rot) {
        setGyroOffset(rot.getDegrees());
        odometry.resetPosition(pose, rot);
        resetEncoderCounts();
    }

    public int getEncoderCount(int sensorIndex) {
        return driveMotors[sensorIndex].getSelectedSensorPosition();
    }

    public double getGyroAngle() {
        return gyro.getAngle();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public void resetGyroAngle() {
        gyro.zeroYaw();
    }

    public void setGyroOffset(double angle) { // Units: Degrees
        gyro.setAngleAdjustment(angle);
    }

    public double getMotorInputCurrent(int motorIndex) {
        return driveMotors[motorIndex].getSupplyCurrent();
    }

    public void resetEncoderCounts() {
        driveMotors[0].setSelectedSensorPosition(0);
        driveMotors[2].setSelectedSensorPosition(0);
    }

    public double getWheelDistanceMeters(int sensorIndex) {
        return (driveMotors[sensorIndex].getSelectedSensorPosition() / 4096.0)
                * Constants.driveGearRatio * Math.PI * Units.feetToMeters(Constants.wheelDiameter);
    }

    public Pose2d getRobotPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveKinematics getDrivetrainKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {
        // Consistently update the robot's odometry as it moves throughout the field
        odometry.update(Rotation2d.fromDegrees(getHeading()), getWheelDistanceMeters(0), getWheelDistanceMeters(2));
    }
}


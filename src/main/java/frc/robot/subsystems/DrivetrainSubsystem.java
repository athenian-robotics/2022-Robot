package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants.maxDriveSpeed;
import static frc.robot.Constants.DriveConstants.minDriveSpeed;

public class DrivetrainSubsystem extends SubsystemBase {
    ChassisSpeeds chassisSpeeds;
    DifferentialDriveOdometry odometry;
    // Creates kinematics object: track width of 27 inches (track width = distance between two sets of wheels)
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);
    public final Encoder rightEncoder;
    public final Encoder leftEncoder;
    private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    private final MotorControllerGroup leftMotors = new MotorControllerGroup(
            new WPI_TalonFX(Constants.DriveConstants.leftFrontDrivePort),
            new WPI_TalonFX(Constants.DriveConstants.leftRearDrivePort));
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(
            new WPI_TalonFX(Constants.DriveConstants.rightFrontDrivePort),
            new WPI_TalonFX(Constants.DriveConstants.rightRearDrivePort));
    private final DifferentialDrive drive;

    public DrivetrainSubsystem() {
        WPI_TalonFX[] driveMotors = {
                new WPI_TalonFX(Constants.DriveConstants.leftFrontDrivePort),
                new WPI_TalonFX(Constants.DriveConstants.leftRearDrivePort),

        };
        configureDriveMotors(driveMotors); // Initialize motors
        //odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading())); // Initialize odometry configuration
        drive = new DifferentialDrive(leftMotors, rightMotors); // Initialize drivetrain
        rightEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k2X);
        leftEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k2X);

        leftEncoder.setDistancePerPulse(6.0 * 0.0254 * Math.PI / 1264); // 6 inch wheel, to meters, 2048 ticks //0.0254
        rightEncoder.setDistancePerPulse(6.0 * 0.0254 * Math.PI / 1264); // 6 inch wheel, to meters, 2048 ticks

        //solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * This method utilizes both joystick outputs on an xbox controller to arcade drive a west-coast drivetrain
     * @param throttle Velocity of drive on the Y axis (Forward & Backwards)
     * @param rot Velocity of rotation
     */
    public void arcadeDrive(double throttle, double rot) {
        double leftPWM = throttle + rot;
        double rightPWM = throttle - rot;

        int leftSign = leftPWM >= 0 ? 1 : -1; // Checks leftSpeed and gathers whether it is negative or positive
        int rightSign = rightPWM >= 0 ? 1 : -1; // Checks rightSpeed and gathers whether it is negative or positive

        // Deadband

        leftPWM = Math.abs(leftPWM) > maxDriveSpeed ? maxDriveSpeed * leftSign : leftPWM;
        rightPWM = Math.abs(rightPWM) > maxDriveSpeed ? maxDriveSpeed * rightSign : rightPWM;

        leftPWM = Math.abs(leftPWM) < minDriveSpeed ? 0 : leftPWM;
        rightPWM = Math.abs(rightPWM) < minDriveSpeed ? 0 : rightPWM;

        setMotorPercentOutput(-leftPWM, rightPWM); // Set motor values based off throttle and rotation inputs
    }

    /**
     * Utilize both joystick values to tank drive a west-coast drivetrain
     * @param leftVelocity Speed of the chassis' left side
     * @param rightVelocity Speed of the chassis' right side
     */
    public void tankDrive(double leftVelocity, double rightVelocity) {
        int leftSign = leftVelocity >= 0 ? 1 : -1; // Checks leftSpeed and gathers whether it is negative or positive
        int rightSign = rightVelocity >= 0 ? 1 : -1; // Checks rightSpeed and gathers whether it is negative or positive

        // Deadband
        leftVelocity = Math.abs(leftVelocity) > maxDriveSpeed ? maxDriveSpeed * leftSign : leftVelocity;
        rightVelocity = Math.abs(rightVelocity) > maxDriveSpeed ? maxDriveSpeed * rightSign : rightVelocity;

        leftVelocity = Math.abs(leftVelocity) < minDriveSpeed ? 0 : leftVelocity;
        rightVelocity = Math.abs(rightVelocity) < minDriveSpeed ? 0 : rightVelocity;

        setMotorPercentOutput(-leftVelocity, rightVelocity);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        leftMotors.setVoltage(rightVolts);
        drive.feed();
    }

    /**
     * Set the front wheels to a desired output. Units: Percentage
     * @param leftOutput Left front wheel output percentage
     * @param rightOutput Right front wheel output percentage
     */
    public void setMotorPercentOutput(double leftOutput, double rightOutput) {
//        driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
//        driveMotors[1].set(ControlMode.PercentOutput, leftOutput);
//        driveMotors[2].set(ControlMode.PercentOutput, rightOutput);
//        driveMotors[3].set(ControlMode.PercentOutput, rightOutput);
        leftMotors.set(leftOutput);
        rightMotors.set(rightOutput);
    }

    /**
     * Factory reset drive motors on initialization
     * @param driveMotors An array of Falcon 500 motors
     */
    public void configureDriveMotors(TalonFX[] driveMotors) {
        for (TalonFX motor: driveMotors) {
            motor.configFactoryDefault(); // Initialize motor set up
            motor.configOpenloopRamp(0.3); // Ramp up (Trapezoid)
            motor.configClosedloopRamp(0.3); // Ramp down (Trapezoid)
            motor.setNeutralMode(NeutralMode.Brake); // Default robot mode should be Coasting
            motor.configForwardSoftLimitEnable(false);
            motor.configReverseSoftLimitEnable(false);
        }
    }

    /**
     * Reset the robot's odometry
     * @param pose Robot's pose as a Pose2d object
     * @param rot Robot's rotation as a Rotation2d object
     */
    public void resetOdometry(Pose2d pose, Rotation2d rot) {
        //setGyroOffset(rot.getDegrees());
        odometry.resetPosition(pose, rot);
        // resetEncoderCounts();
    }

    public int getLeftEncoderCount() { return this.leftEncoder.get(); } // Returns left encoder raw count

    public int getRightEncoderCount() { return this.rightEncoder.get(); } // Returns right encoder raw count

    public void resetEncoderCounts() { // Resets the encoders
        this.leftEncoder.reset();
        this.rightEncoder.reset();
    }

    public double getRightDistanceDriven() { return rightEncoder.getDistance(); } // Returns the distance the right side has driven

    public double getLeftDistanceDriven() { return leftEncoder.getDistance(); } // Returns the distance the left side has driven

    public double getGyroAngle() {
        return gyro.getAngle(); // Returns gyro angle
    }

    public void resetGyroAngle() {
        gyro.zeroYaw(); // Zero Gyro's angle
    }

    public void setGyroOffset(double angle) { // Units: Degrees
        gyro.setAngleAdjustment(angle); // Sets the gyro's offset
    }

    public double getHeading() { return Math.IEEEremainder(-gyro.getAngle(), 360); } // Gets the gyro's heading, scaled within 360 degrees

    public void toggleShifter() { solenoid.toggle(); }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    private void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }


    @Override
    public void periodic() {
        // Consistently update the robot's odometry as it moves throughout the field

        SmartDashboard.putNumber("Gyro Angle: ", gyro.getAngle());
        SmartDashboard.putNumber("Gyro Yaw: ", gyro.getYaw());
        SmartDashboard.putNumber("Raw Left Enc: ", getLeftEncoderCount());
        SmartDashboard.putNumber("Raw Right Enc: ", getRightEncoderCount());
        SmartDashboard.putNumber("Left Dist: ", getLeftDistanceDriven());
        SmartDashboard.putNumber("Right Dist: ", getRightDistanceDriven());
    }


    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }
}


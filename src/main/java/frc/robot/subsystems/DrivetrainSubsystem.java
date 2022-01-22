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
import static frc.robot.Constants.PneumaticConstants.shifterSolenoidPortA;
import static frc.robot.Constants.PneumaticConstants.shifterSolenoidPortB;

public class DrivetrainSubsystem extends SubsystemBase {
    ChassisSpeeds chassisSpeeds;
    DifferentialDriveOdometry odometry;
    // Creates kinematics object: track width of 27 inches (track width = distance between two sets of wheels)
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);
    public final Encoder rightEncoder;
    public final Encoder leftEncoder;
    private final DoubleSolenoid driveShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, shifterSolenoidPortA, shifterSolenoidPortB);

    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;
    private final DifferentialDrive drive;

    public DrivetrainSubsystem() {
        WPI_TalonFX[] driveMotors = {
                new WPI_TalonFX(Constants.DriveConstants.rightRearDrivePort),
                new WPI_TalonFX(Constants.DriveConstants.rightFrontDrivePort),
                new WPI_TalonFX(Constants.DriveConstants.leftRearDrivePort),
                new WPI_TalonFX(Constants.DriveConstants.leftFrontDrivePort)
        };
        configureDriveMotors(driveMotors); // Initialize motors
        leftMotors = new MotorControllerGroup(driveMotors[0], driveMotors[1]);
        rightMotors = new MotorControllerGroup(driveMotors[2], driveMotors[3]);
        rightMotors.setInverted(true);
        drive = new DifferentialDrive(leftMotors, rightMotors); // Initialize Differential Drive

        rightEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k2X);
        leftEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k2X);
        leftEncoder.setDistancePerPulse(6.0 * 0.0254 * Math.PI / 2048 * 4/3); // 6 inch wheel, to meters, 2048 ticks //0.0254
        rightEncoder.setDistancePerPulse(6.0 * 0.0254 * Math.PI / 2048 * 4/3);// 6 inch wheel, to meters, 2048 ticks

        driveShifter.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * This method utilizes both joystick outputs on an xbox controller to arcade drive a west-coast drivetrain
     * @param throttle Velocity of drive on the Y axis (Forward & Backwards)
     * @param rot Velocity of rotation
     */
    public void arcadeDrive(double throttle, double rot) {
        drive.arcadeDrive(throttle * maxDriveSpeed, maxDriveSpeed < 0 ? -rot * maxDriveSpeed : rot * maxDriveSpeed);
    }

    /**
     * Utilize both joystick values to tank drive a west-coast drivetrain
     * @param leftVelocity Speed of the chassis' left side
     * @param rightVelocity Speed of the chassis' right side
     */
    public void tankDrive(double leftVelocity, double rightVelocity) {
        int leftSign = leftVelocity >= 0 ? 1 : -1; // Checks leftSpeed and gathers whether it is negative or positive
        int rightSign = rightVelocity >= 0 ? 1 : -1; // Checks rightSpeed and gathers whether it is negative or positive

        double leftPower = ((maxDriveSpeed - minDriveSpeed) * Math.abs(leftVelocity) + minDriveSpeed) * leftSign;
        double rightPower = ((maxDriveSpeed - minDriveSpeed) * Math.abs(rightVelocity) + minDriveSpeed) * rightSign;

        drive.tankDrive(leftVelocity, rightVelocity);
    }

    /**
     * Tank drive without deadband, giving full control of velocities to PID controller
     * @param leftVelocity Velocity of the left wheels
     * @param rightVelocity Velocity of the right wheels
     */
    public void autoTankDrive(double leftVelocity, double rightVelocity) {
        setMotorPercentOutput(leftVelocity, rightVelocity);
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
            motor.configOpenloopRamp(0.2); // Ramp up (Trapezoid)
            motor.configClosedloopRamp(0.2); // Ramp down (Trapezoid)
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
        return gyro.getAngle() % 360; // Returns gyro angle
    }

    public void resetGyro() {
        gyro.reset(); // Zero Gyro's angle
    }

    public void setGyroOffset(double angle) { // Units: Degrees
        gyro.setAngleAdjustment(angle); // Sets the gyro's offset
    }

    public double getHeading() { return Math.IEEEremainder(-gyro.getAngle(), 360); } // Gets the gyro's heading, scaled within 360 degrees

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    private void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void toggleShifter() { driveShifter.toggle(); }

    @Override
    public void periodic() {
        // Consistently update the robot's odometry as it moves throughout the field

        SmartDashboard.putNumber("Gyro Angle: ", getGyroAngle());
        SmartDashboard.putNumber("Gyro Heading: ", getHeading());
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


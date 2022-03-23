package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.DriveConstants.*;


public class DrivetrainSubsystem extends SubsystemBase {
    // Setup drive objects
    public final Encoder rightEncoder;
    public final Encoder leftEncoder;
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);
    private final DoubleSolenoid driveShifterRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.PneumaticConstants.shifterRightSolenoidPortA,
            Constants.PneumaticConstants.shifterRightSolenoidPortB);
    private final DoubleSolenoid driveShifterLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.PneumaticConstants.shifterLeftSolenoidPortA,
            Constants.PneumaticConstants.shifterLeftSolenoidPortB);
    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;
    private final DifferentialDrive drive;
    // Setup autonomous and sensor objects
    ChassisSpeeds chassisSpeeds;
    DifferentialDriveOdometry odometry;

    public DrivetrainSubsystem() {
        // Initialize motors
        WPI_TalonFX[] driveMotors = {
                new WPI_TalonFX(4),
                new WPI_TalonFX(3),
                new WPI_TalonFX(1),
                new WPI_TalonFX(2)
        };
        configureDriveMotors(driveMotors); // Configure motors

        leftMotors = new MotorControllerGroup(driveMotors[0], driveMotors[1]);
        rightMotors = new MotorControllerGroup(driveMotors[2], driveMotors[3]);
        leftMotors.setInverted(true);

        drive = new DifferentialDrive(leftMotors, rightMotors); // Initialize Differential Drive

        // Configure encoders
        rightEncoder = new Encoder(rightEncoderChannelA, rightEncoderchannelB, true);
        leftEncoder = new Encoder(leftEncoderChannelA, leftEncoderChannelB, false);
        leftEncoder.setDistancePerPulse(2 * 3.14 * (.1524 / 2) / 2048); // 6-inch wheel, to meters, PI for
        // circumference, gear conversion, 2048 ticks per rotation
        rightEncoder.setDistancePerPulse(2 * 3.14 * (.1524 / 2) / 2048) ; // 6-inch wheel, to meters, PI for
        // circumference, gear conversion, 2048 ticks per rotation

        //Configure solenoids
        driveShifterRight.set(DoubleSolenoid.Value.kReverse);
        driveShifterLeft.set(DoubleSolenoid.Value.kReverse);

        resetEncoders();
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    /**
     * This method utilizes both joystick outputs on an xbox controller to arcade drive a west-coast drivetrain
     *
     * @param throttle Velocity of drive on the Y axis (Forward & Backwards)
     * @param rot      Velocity of rotation
     */
    public void arcadeDrive(double throttle, double rot) {
        drive.arcadeDrive(throttle * maxDriveSpeed, rot * maxDriveSpeed);
    }

    /**
     * Utilize both joystick values to tank drive a west-coast drivetrain
     *
     * @param leftVelocity  Speed of the chassis' left side
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
     *
     * @param leftVelocity  Velocity of the left wheels
     * @param rightVelocity Velocity of the right wheels
     */
    public void autoTankDrive(double leftVelocity, double rightVelocity) {
        setMotorPercentOutput(leftVelocity, rightVelocity);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drive.feed();
    }

    /**
     * Set the front wheels to a desired output. Units: Percentage
     *
     * @param leftOutput  Left front wheel output percentage
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
     *
     * @param driveMotors An array of Falcon 500 motors
     */
    public void configureDriveMotors(TalonFX[] driveMotors) {
        for (TalonFX motor : driveMotors) {
            motor.configFactoryDefault(); // Initialize motor set up
            motor.configOpenloopRamp(0.7); // Ramp up (Trapezoid)
            motor.configClosedloopRamp(0.7); // Ramp down (Trapezoid)
            motor.setNeutralMode(NeutralMode.Brake); // Default robot mode should be Coasting (So it doesn't wobble
            // cuz top heavy yaknow)
            motor.configForwardSoftLimitEnable(false);
            motor.configReverseSoftLimitEnable(false);
        }
    }

    /**
     * Reset the robot's odometry
     *
     * @param pose Robot's pose as a Pose2d object
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public int getLeftEncoderCount() {return this.leftEncoder.get();} // Returns left encoder raw count

    public int getRightEncoderCount() {return this.rightEncoder.get();} // Returns right encoder raw count

    public double getRightDistanceDriven() {return rightEncoder.getDistance();} // Returns the distance the right
    // side has driven

    public double getLeftDistanceDriven() {return leftEncoder.getDistance();} // Returns the distance the left side
    // has driven

    public double getGyroAngle() {
        return gyro.getAngle() % 360;
    } // Returns the total accumulated yaw scaled under 360

    public double getGyroYaw() {
        return gyro.getYaw();
    } // Returns the gyro's rotation about the Z-axis

    public void resetGyro() {gyro.reset();} // Zero Gyro's angle

    public void setGyroOffset(double angle) {gyro.setAngleAdjustment(angle);}//  Sets the gyro's offset (units: degrees)

    public double getHeading() {return Math.IEEEremainder(-gyro.getAngle(), 360);} // Gets the gyro's heading, scaled

    private void resetEncoders() { // Resets the drive encoders
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void toggleShifter() { // Toggles drive shifters
        driveShifterRight.toggle();
        driveShifterLeft.toggle();
    }

    public void shiftUp() { // Shifts up drive shifters
        driveShifterRight.set(DoubleSolenoid.Value.kForward);
        driveShifterLeft.set(DoubleSolenoid.Value.kForward);
    }

    public void shiftDown() { // Shift down drive shifters
        driveShifterRight.set(DoubleSolenoid.Value.kReverse);
        driveShifterLeft.set(DoubleSolenoid.Value.kReverse);
    }

    public Pose2d getPose() { // Returns the Pose2d object of the robot in meters
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() { // Returns the differential drive wheel speeds of the chassis
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    public void disable() { // Disables drivetrain movement
        drive.tankDrive(0, 0);
    }

    /**
     * Periodic will be run over and over again, similar to that of a command's execute method, when the subsystem is
     * initialized
     */
    @Override
    public void periodic() {
        // Consistently update the robot's odometry as it moves throughout the field
        SmartDashboard.putNumber("Gyro Angle: ", getGyroAngle());
        SmartDashboard.putNumber("Gyro Heading: ", getHeading());
        SmartDashboard.putNumber("Gyro Yaw: ", getGyroYaw());
        SmartDashboard.putNumber("Left Drive Encoder: ", getLeftEncoderCount());
        SmartDashboard.putNumber("Right Drive Encoder: ", getRightEncoderCount());
        SmartDashboard.putNumber("Left Drive Distance: ", getLeftDistanceDriven());
        SmartDashboard.putNumber("Right Drive Distance: ", getRightDistanceDriven());

        odometry.update(gyro.getRotation2d(),
                leftEncoder.getDistance(),
                rightEncoder.getDistance());

    }
}


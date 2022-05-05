package frc.robot.subsystems;

import static frc.robot.Constants.AutoConstants.kDriveKinematics;
import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.motors.TalonFXFactory;

public class DrivetrainSubsystem extends SubsystemBase {
  // Setup drive objects
  public final Encoder rightEncoder;
  public final Encoder leftEncoder;
  // Setup autonomous and sensor objects
  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  private final DoubleSolenoid driveShifterRight =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          Constants.PneumaticConstants.shifterRightSolenoidPortA,
          Constants.PneumaticConstants.shifterRightSolenoidPortB);
  private final DoubleSolenoid driveShifterLeft =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          Constants.PneumaticConstants.shifterLeftSolenoidPortA,
          Constants.PneumaticConstants.shifterLeftSolenoidPortB);
  private final MotorControllerGroup leftMotors;
  private final MotorControllerGroup rightMotors;
  private final DifferentialDrive drive;
  private final WPI_TalonFX[] driveMotors;

  public DrivetrainSubsystem() {
    // Initialize motors
    driveMotors =
        new WPI_TalonFX[] {
          TalonFXFactory.createDefaultTalon(rightRearDrivePort),
          TalonFXFactory.createDefaultTalon(rightFrontDrivePort),
          TalonFXFactory.createDefaultTalon(leftRearDrivePort),
          TalonFXFactory.createDefaultTalon(leftFrontDrivePort)
        };
    configureDriveMotors(driveMotors); // Configure motors

    leftMotors = new MotorControllerGroup(driveMotors[0], driveMotors[1]);
    rightMotors = new MotorControllerGroup(driveMotors[2], driveMotors[3]);
    leftMotors.setInverted(true);

    drive = new DifferentialDrive(leftMotors, rightMotors); // Initialize Differential Drive

    // Configure encoders
    rightEncoder = new Encoder(rightEncoderChannelA, rightEncoderChannelB, true);
    leftEncoder = new Encoder(leftEncoderChannelA, leftEncoderChannelB, false);
    leftEncoder.setDistancePerPulse(
        2 * 3.14 * (.1524 / 2) / 2048); // 6-inch wheel, to meters, PI for
    // circumference, gear conversion, 2048 ticks per rotation
    rightEncoder.setDistancePerPulse(
        2 * 3.14 * (.1524 / 2) / 2048); // 6-inch wheel, to meters, PI for
    // circumference, gear conversion, 2048 ticks per rotation

    // Configure solenoids
    driveShifterRight.set(DoubleSolenoid.Value.kReverse);
    driveShifterLeft.set(DoubleSolenoid.Value.kReverse);

    resetEncoders();
  }

  /**
   * This method utilizes both joystick outputs on an xbox controller to arcade drive a west-coast
   * drivetrain
   *
   * @param throttle Velocity of drive on the Y axis (Forward & Backwards)
   * @param rot Velocity of rotation
   */
  public void arcadeDrive(double throttle, double rot) {
    drive.arcadeDrive(throttle * maxDriveSpeed, rot * maxDriveSpeed);
  }

  /**
   * Utilize both joystick values to tank drive a west-coast drivetrain
   *
   * @param leftVelocity Speed of the chassis' left side
   * @param rightVelocity Speed of the chassis' right side
   */
  public void tankDrive(double leftVelocity, double rightVelocity) {
    int leftSign =
        leftVelocity >= 0
            ? 1
            : -1; // Checks leftSpeed and gathers whether it is negative or positive
    int rightSign =
        rightVelocity >= 0
            ? 1
            : -1; // Checks rightSpeed and gathers whether it is negative or positive

    double leftPower =
        ((maxDriveSpeed - minDriveSpeed) * Math.abs(leftVelocity) + minDriveSpeed) * leftSign;
    double rightPower =
        ((maxDriveSpeed - minDriveSpeed) * Math.abs(rightVelocity) + minDriveSpeed) * rightSign;

    drive.tankDrive(leftPower, rightPower);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    drive.feed();
  }

  /**
   * Factory reset drive motors on initialization
   *
   * @param driveMotors An array of Falcon 500 motors
   */
  public void configureDriveMotors(TalonFX[] driveMotors) {
    for (TalonFX motor : driveMotors) {
      motor.configFactoryDefault(); // Initialize motor set up
      motor.configOpenloopRamp(0.6); // Ramp up (Trapezoid)
      motor.configClosedloopRamp(0.55); // Ramp down (Trapezoid)
      motor.setNeutralMode(
          NeutralMode.Brake); // Default robot mode should be Coasting (So it doesn't wobble
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
    RobotContainer.poseEstimator.resetPose(pose);
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public double getRightDistanceDriven() {
    return rightEncoder.getDistance();
  } // Returns the distance the right
  // side has driven

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
    for (WPI_TalonFX motor : driveMotors) {
      motor.configOpenloopRamp(0.8);
      motor.configClosedloopRamp(1.1);
    }
  }

  public void shiftDown() { // Shift down drive shifters
    driveShifterRight.set(DoubleSolenoid.Value.kReverse);
    driveShifterLeft.set(DoubleSolenoid.Value.kReverse);
    for (WPI_TalonFX motor : driveMotors) {

      motor.configClosedloopRamp(0.64);
      motor.configOpenloopRamp(0.57);
    }
  }

  public Pose2d getPose() { // Returns the Pose2d object of the robot in meters
    return RobotContainer.poseEstimator.getPose();
  }

  public DifferentialDriveWheelSpeeds
      getWheelSpeeds() { // Returns the differential drive wheel speeds of the chassis
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public void disable() { // Disables drivetrain movement
    drive.stopMotor();
  }

  /**
   * Periodic will be run over and over again, similar to that of a command's execute method, when
   * the subsystem is initialized
   */
  @Override
  public void periodic() {
    // Consistently update the robot's odometry as it moves throughout the field
    drive.feed();
  }

  public double getLeftDistanceDriven() {
    return leftEncoder.getDistance();
  }

  public double getPositionOffset() {
    Pose2d pose = RobotContainer.limelight.getLatestPose().pose;
    double velocity = kDriveKinematics.toChassisSpeeds(getWheelSpeeds()).vxMetersPerSecond;
    double distance = pose.getTranslation().getNorm();
    double TOF = RobotContainer.shooterDataTable.getSpecs(Math.abs(distance)).getTOF();
    Translation2d displacementVector = new Pose2d().minus(pose).getTranslation();
    Translation2d linearVelocityVector = new Translation2d(velocity, new Rotation2d(0));
    double side = TOF * velocity;
    double tofOffset = Math.atan(side / distance);
    if (pose.getRotation().getDegrees() < 180) {
      tofOffset *= -1;
    }
    double angularVelocity =
        kDriveKinematics.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond;
    // cross product
    double cross =
        displacementVector.getX() * linearVelocityVector.getY()
            - displacementVector.getY() * linearVelocityVector.getX();
    return ((cross + angularVelocity) * Constants.looptime) - tofOffset;
  }

  public static double getPositionOffset(
      double angle, double distance, double velocity, double TOF, double angularVelocity) {
    Translation2d displacementVector =
        new Translation2d(distance, new Rotation2d(Math.toRadians(-angle)));
    Translation2d linearVelocityVector = new Translation2d(velocity, new Rotation2d(0));

    double side = TOF * velocity;
    double tofOffset = Math.atan(side / distance);
    if (angle < -180) {
      tofOffset *= -1;
    }
    // cross product
    double cross =
        displacementVector.getX() * linearVelocityVector.getY()
            - displacementVector.getY() * linearVelocityVector.getX();

    double v = ((cross + angularVelocity) * Constants.looptime) - tofOffset;
    SmartDashboard.putNumber("tof + turret offset", v);
    return v;
  }

  public static void main(String[] args) {
    System.out.println(Math.toDegrees(getPositionOffset(-90, 5, 1, 1.46, 0)));
  }
}

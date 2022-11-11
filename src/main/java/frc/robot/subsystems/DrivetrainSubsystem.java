package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.motors.TalonFXFactory;

public class DrivetrainSubsystem extends SubsystemBase {
  // Setup drive objects
  // Setup autonomous and sensor objects
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

    // Configure solenoids
    driveShifterRight.set(DoubleSolenoid.Value.kReverse);
    driveShifterLeft.set(DoubleSolenoid.Value.kReverse);
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

  public void curvatureDrive(double throttle, double rot) {
    throttle = Math.copySign(throttle * throttle, throttle);
    rot = Math.copySign(rot * rot, rot);
    drive.curvatureDrive(throttle, rot, true);
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
      motor.configOpenloopRamp(0.35); // Ramp up (Trapezoid)
      motor.configClosedloopRamp(0.35); // Ramp down (Trapezoid)
      motor.setNeutralMode(
          NeutralMode.Brake); // Default robot mode should be Coasting (So it doesn't wobble
      // cuz top heavy yaknow)
      motor.configForwardSoftLimitEnable(false);
      motor.configReverseSoftLimitEnable(false);
    }
  }

  public void shiftUp() { // Shifts up drive shifters
    driveShifterRight.set(DoubleSolenoid.Value.kForward);

    driveShifterLeft.set(DoubleSolenoid.Value.kForward);
    for (WPI_TalonFX motor : driveMotors) {
      motor.configOpenloopRamp(0.35);
      motor.configClosedloopRamp(.35);
    }
  }

  public void shiftDown() { // Shift down drive shifters
    driveShifterRight.set(DoubleSolenoid.Value.kReverse);
    driveShifterLeft.set(DoubleSolenoid.Value.kReverse);
    for (WPI_TalonFX motor : driveMotors) {
      motor.configClosedloopRamp(0.82);
      motor.configOpenloopRamp(0.77);
    }
  }

  public void disable() { // Disables drivetrain movement
    drive.stopMotor();
  }

  @Override
  public void periodic() {
    drive.feed();
  }
}

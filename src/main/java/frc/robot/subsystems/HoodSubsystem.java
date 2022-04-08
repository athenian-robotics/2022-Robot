package frc.robot.subsystems;

import static frc.robot.Constants.MechanismConstants.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class HoodSubsystem extends SubsystemBase {
  private final Servo leftHoodAngleServo = new Servo(leftHoodServoPort);
  private final Servo rightHoodAngleServo = new Servo(rightHoodServoPort);
  private final NetworkTableEntry hoodAngleNTE;
  public double lastHoodAngle;

  public HoodSubsystem() {
    leftHoodAngleServo.setBounds(
        2.0, 1.8, 1.5, 1.2, 1.0); // Manufacturer specified for Actuonix linear servos
    rightHoodAngleServo.setBounds(
        2.0, 1.8, 1.5, 1.2, 1.0); // Manufacturer specified for Actuonix linear servos

    hoodAngleNTE =
        Shuffleboard.getTab("852 - Dashboard")
            .add("Hood Angle 2", 1)
            .withWidget(BuiltInWidgets.kTextView)
            .withProperties(Map.of("min", 8, "max", 41))
            .getEntry();
  }

  public void setHoodAngle(double angle) {
    lastHoodAngle = getHoodAngle();
    if (angle >= minimumHoodAngle && angle <= maximumHoodAngle) {
      leftHoodAngleServo.setAngle(
          180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle));
      // 0 - 180 DEGREES
      rightHoodAngleServo.setAngle(
          180 * (angle - minimumHoodAngle) / (maximumHoodAngle - minimumHoodAngle));
      // 0 - 180 DEGREES
    }
  }

  public double
      getHoodAngle() { // Takes the average of the angles (0-1) and scales it into a degree
    // measurement
    return ((maximumHoodAngle - minimumHoodAngle)
            * (leftHoodAngleServo.getAngle() + rightHoodAngleServo.getAngle())
            / 360)
        + minimumHoodAngle;
  }

  public void disable() {
    leftHoodAngleServo.setSpeed(0);
    rightHoodAngleServo.setSpeed(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle", getHoodAngle());
    SmartDashboard.putNumber("lastHoodAngle", lastHoodAngle);

    // setHoodAngle(hoodAngleNTE.getDouble(31));
  }
}

package frc.robot.subsystems;

import static frc.robot.Constants.MechanismConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.lib.shooterData.ShooterDataTable;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Map;

public class HoodSubsystem extends SubsystemBase implements Loggable {
  private final NetworkTableEntry fuckLeo;
  private final Servo leftHoodAngleServo = new Servo(leftHoodServoPort);
  private final Servo rightHoodAngleServo = new Servo(rightHoodServoPort);
  private final ShooterDataTable table;
  private final LimelightSubsystem limelight;
  @Log public double lastHoodAngle;
  private double HUB_ANGLE = 0; // TODO: test
  @Log private double setpoint;
  @Log private double logClamped;

  public HoodSubsystem(ShooterDataTable table, LimelightSubsystem limelight) {
    this.table = table;
    leftHoodAngleServo.setBounds(
        2.0, 1.8, 1.5, 1.2, 1.0); // Manufacturer specified for Actuonix linear servos
    rightHoodAngleServo.setBounds(
        2.0, 1.8, 1.5, 1.2, 1.0); // Manufacturer specified for Actuonix linear servos

    fuckLeo =
        Shuffleboard.getTab("852 - Dashboard")
            .add("fuck", 25)
            .withProperties(Map.of("min", 25, "max", 33.9, "default value", 25))
            .getEntry();
    this.limelight = limelight;
  }

  public void setHoodAngle(double angle) {
    // angle is in degrees between 25 and 33.9, the input to the servo is a value between 41 and 105
    double normAngle = 7.19101 * angle - 138.775; // dont ask
    double clampedAngle = MathUtil.clamp(normAngle, 41, 105);
    logClamped = clampedAngle;
    leftHoodAngleServo.setAngle(clampedAngle);
    rightHoodAngleServo.setAngle(clampedAngle);
  }

  public void setHoodAngleRaw(double angle) {
    rightHoodAngleServo.setAngle(angle);
    leftHoodAngleServo.setAngle(angle);
  }

  public Command approachTarget() {
    if (!limelight.isTarget()) return new InstantCommand(() -> {});
    return new InstantCommand(
        () -> setHoodAngle(table.getSpecs(limelight.getDistance()).getAngle()), this, limelight);
  }

  public void disable() {
    leftHoodAngleServo.setSpeed(0);
    rightHoodAngleServo.setSpeed(0);
  }

  public Command hoodHub() {
    return new InstantCommand(
        () -> setHoodAngle(HUB_ANGLE),
        this);
  }

  public Command test() {
    return new InstantCommand(() -> setHoodAngle(fuckLeo.getDouble(25)));
  }
}

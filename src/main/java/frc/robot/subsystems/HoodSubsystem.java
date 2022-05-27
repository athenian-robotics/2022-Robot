package frc.robot.subsystems;

import static frc.robot.Constants.MechanismConstants.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.lib.shooterData.ShooterSpec;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Map;

public class HoodSubsystem extends SubsystemBase {
  private final NetworkTableEntry hoodAngleAdjustmentNTE;
  private final NetworkTableEntry fuckLeo;
  private final Servo leftHoodAngleServo = new Servo(leftHoodServoPort);
  private final Servo rightHoodAngleServo = new Servo(rightHoodServoPort);
  private final ShooterDataTable table;
  private final LimelightSubsystem limelight;
  @Log public double lastHoodAngle;

  public HoodSubsystem(ShooterDataTable table, LimelightSubsystem limelight) {
    this.table = table;
    leftHoodAngleServo.setBounds(
        2.0, 1.8, 1.5, 1.2, 1.0); // Manufacturer specified for Actuonix linear servos
    rightHoodAngleServo.setBounds(
        2.0, 1.8, 1.5, 1.2, 1.0); // Manufacturer specified for Actuonix linear servos

    hoodAngleAdjustmentNTE =
        Shuffleboard.getTab("852 - Dashboard")
            .add("Hood Angle Adjustment", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.75, "max", 1.25, "default value", 1))
            .getEntry();
    fuckLeo =
        Shuffleboard.getTab("852 - Dashboard")
            .add("fuck", 33)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 8, "max", 41, "default value", 33))
            .getEntry();
    this.limelight = limelight;
  }

  @Log
  public double getHoodAngle() {
    return Math.toRadians(
        ((maximumHoodAngle - minimumHoodAngle)
                * (leftHoodAngleServo.getAngle() + rightHoodAngleServo.getAngle())
                / 360)
            + minimumHoodAngle);
  }

  public void setHoodAngle(double angle) {
    lastHoodAngle = getHoodAngle();
    if (angle >= minimumHoodAngle && angle <= maximumHoodAngle) {
      leftHoodAngleServo.setAngle(
          hoodAngleAdjustmentNTE.getDouble(1)
              * 180
              * (angle - minimumHoodAngle)
              / (maximumHoodAngle - minimumHoodAngle));
      // 0 - 180 DEGREES
      rightHoodAngleServo.setAngle(
          hoodAngleAdjustmentNTE.getDouble(1)
              * 180
              * (angle - minimumHoodAngle)
              / (maximumHoodAngle - minimumHoodAngle));
      // 0 - 180 DEGREES
    }
  }

  public boolean atLimelightSetpoint() {
    return Math.abs(
            getHoodAngle()
                - table
                    .getSpecs(
                        limelight.getTrackedTarget().getCameraToTarget().getTranslation().getNorm())
                    .getAngle())
        < 0.75;
  }

  public Command approachTarget() {
    double distance = limelight.getDistance();
    ShooterSpec fuck = table.getSpecs(distance);
    return new InstantCommand(
        () -> setHoodAngle(table.getSpecs(limelight.getDistance()).getAngle()), this);
  }

  public Command waitUntilSetpoint() {
    return new WaitUntilCommand(this::atLimelightSetpoint);
  }

  public void disable() {
    leftHoodAngleServo.setSpeed(0);
    rightHoodAngleServo.setSpeed(0);
  }
}

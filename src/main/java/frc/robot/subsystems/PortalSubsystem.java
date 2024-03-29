package frc.robot.subsystems;

import static frc.robot.Constants.MechanismConstants.intakeToIndexerMotorPort;
import static frc.robot.RobotContainer.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.colorwheel.ColorWheelUtils;
import frc.robot.lib.colorwheel.WheelColors;

public class PortalSubsystem extends SubsystemBase {
  private final CANSparkMax portalMotor =
      new CANSparkMax(intakeToIndexerMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final ColorWheelUtils colorWheelUtils = new ColorWheelUtils();

  public WheelColors currentColor = WheelColors.GREEN;
  public double currentProximity = 0;

  public PortalSubsystem() {
    portalMotor.setInverted(true);
  }

  public Command startPortal() {
    return new InstantCommand(this::run);
  }

  public void startPortalInverted() {
    portalMotor.set(-Constants.MechanismConstants.intakeToIndexerSpeed);
  }

  public Command stopPortal() {
    return new InstantCommand(this::stop);
  }

  public boolean ballPrimed() {
    return currentProximity > 10;
  }

  public DriverStation.Alliance primedBallColor() {
    if (ballPrimed()) {
      if (currentColor == WheelColors.RED) return DriverStation.Alliance.Red;
      if (currentColor == WheelColors.BLUE) return DriverStation.Alliance.Blue;
    }
    return DriverStation.Alliance.Invalid;
  }

  public boolean allianceBallPrimed() {
    return ballPrimed()
        && primedBallColor() == RobotContainer.alliance
        && RobotContainer.alliance != DriverStation.Alliance.Invalid;
  }

  public void disable() {
    stopPortal();
  }

  @Override
  public void periodic() {
    currentColor = colorWheelUtils.currentColor();
    currentProximity = colorWheelUtils.currentProximity();
    if (ballPrimed() && !indexer.fuckLeoButNotInASexualMannerMoreOfADerogatoryone) stop();
    SmartDashboard.putBoolean("Ball Primed", ballPrimed());
  }

  public void run() {
    portalMotor.set(Constants.MechanismConstants.intakeToIndexerSpeed);
  }

  public void stop() {
    portalMotor.set(0);
  }
}

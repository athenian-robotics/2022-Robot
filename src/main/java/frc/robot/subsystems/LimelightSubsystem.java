package frc.robot.subsystems;

import static java.lang.Double.NaN;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class LimelightSubsystem extends SubsystemBase implements Loggable {
  private PhotonTrackedTarget target;
  private PhotonCamera limelight;
  private NetworkTableEntry mmom;

  @Log private double timeSinceLastUpdate;

  public LimelightSubsystem() {
    limelight = new PhotonCamera("gloworm");
    mmom =
        Shuffleboard.getTab("852 - Dashboard")
            .add("can shoot", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
  }

  @Override
  public void periodic() {
    timeSinceLastUpdate =
        Timer.getFPGATimestamp() - limelight.getLatestResult().getLatencyMillis() * 1000;
    if (limelight.getLatestResult().hasTargets())
      target = limelight.getLatestResult().getBestTarget();
    mmom.setBoolean(getDistance() >= 1.53);
  }

  public Optional<PhotonTrackedTarget> getTrackedTarget() {
    return Optional.ofNullable(target);
  }

  @Log.BooleanBox
  public boolean isTarget() {
    return limelight.getLatestResult().hasTargets() || getTrackedTarget().isPresent();
  }

  @Log
  public double getDistance() {
    Optional<PhotonTrackedTarget> target = getTrackedTarget();
    if (target.isEmpty()) return NaN;
    double yaw = target.get().getPitch();
    return (2.6416 - 0.71) / Math.tan(Math.toRadians(40 + yaw));
  }

  public double getTimeSinceLastUpdate() {
    return timeSinceLastUpdate;
  }

  // i hate limelightdatalatch
}

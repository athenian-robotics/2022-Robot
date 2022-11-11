package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class LimelightSubsystem extends SubsystemBase implements Loggable {
  private PhotonTrackedTarget target;
  private PhotonCamera limelight;
  @Log private double timeSinceLastUpdate;

  public LimelightSubsystem() {
    limelight = new PhotonCamera("gloworm");
  }

  @Override
  public void periodic() {
    timeSinceLastUpdate =
        Timer.getFPGATimestamp() - limelight.getLatestResult().getLatencyMillis() * 1000;
    target = limelight.getLatestResult().getBestTarget();
  }

  public PhotonTrackedTarget getTrackedTarget() {
    return target == null ? new PhotonTrackedTarget() : target;
  }

  @Log
  public double getDistance() {
    PhotonTrackedTarget target = getTrackedTarget();
    double yaw = target.getPitch();
    return (2.6416 - 0.71) / Math.tan(Math.toRadians(40 + yaw));
  }

  public double getTimeSinceLastUpdate() {
    return timeSinceLastUpdate;
  }

  // i hate limelightdatalatch
}

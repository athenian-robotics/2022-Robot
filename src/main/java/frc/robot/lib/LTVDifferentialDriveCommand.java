package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class LTVDifferentialDriveCommand extends CommandBase {
  private final LTVDifferentialDriveController controller;
  private final Timer timer = new Timer();
  private final Trajectory trajectory;
  private final Supplier<Pose2d> pose;
  private final DifferentialDriveKinematics kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> speed;
  private final BiConsumer<Double, Double> output;
  private DifferentialDriveWheelSpeeds prevSpeeds;
  private double prevTime;

  public LTVDifferentialDriveCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      LTVDifferentialDriveController controller,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
      BiConsumer<Double, Double> outputVolts,
      Subsystem... requirements) {

    this.trajectory = trajectory;
    this.pose = pose;
    this.kinematics = kinematics;
    this.speed = wheelSpeeds;
    this.output = outputVolts;
    this.controller = controller;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    Trajectory.State initialState = trajectory.sample(0);
    prevSpeeds =
        kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    prevTime = -1;
  }

  @Override
  public void execute() {

    double curTime = timer.get();
    double dt = curTime - prevTime;

    if (prevTime < 0) {
      output.accept(0.0, 0.0);
      prevTime = curTime;
      return;
    }

    Trajectory.State trajRef = trajectory.sample(curTime);
    double leftVelo = speed.get().leftMetersPerSecond;
    double rightVelo = speed.get().rightMetersPerSecond;

    var wheelVolts =
        controller.calculate(pose.get(), leftVelo, rightVelo, trajRef);

    output.accept(wheelVolts.left, wheelVolts.right);
    prevTime = curTime;
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();

    if (interrupted) {
      output.accept(0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}

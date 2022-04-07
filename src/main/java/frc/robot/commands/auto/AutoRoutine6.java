package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants;

import static frc.robot.Constants.AutoConstants.maxAutoAcceleration;
import static frc.robot.Constants.AutoConstants.maxAutoSpeed;


//generalized auto movement command :)
public class AutoRoutine6 extends CommandBase {
  private final RamseteCommand ramseteCommand;
  private final PathPlannerTrajectory trajectory;
  private final DrivetrainSubsystem drivetrain;
  private final boolean resetOdomety;

  public AutoRoutine6(DrivetrainSubsystem drivetrainSubsystem, String pathName, double maxVel, double maxAccel, boolean resetOdometry) {
    this.drivetrain = drivetrainSubsystem;
    this.resetOdomety = resetOdometry;
    addRequirements(drivetrainSubsystem);

    //load path from name
    trajectory = PathPlanner.loadPath(pathName, Math.min(maxVel, maxAutoSpeed), Math.min(maxAccel, maxAutoAcceleration));
    ramseteCommand =
        new RamseteCommand(
            trajectory,
            drivetrainSubsystem::getPose,
            new RamseteController(
                Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.AutoConstants.ksVolts,
                Constants.AutoConstants.kvVoltSecondsPerMeter,
                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
            Constants.AutoConstants.kDriveKinematics,
            drivetrainSubsystem::getWheelSpeeds,
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.AutoConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrainSubsystem::tankDriveVolts,
            drivetrainSubsystem);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    if (resetOdomety) drivetrain.resetOdometry(trajectory.getInitialPose()); // Reset odometry to the starting pose of the trajectory.
    ramseteCommand.initialize();
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute() {
    ramseteCommand.execute();
  }

  /**
   * Returns whether this command has finished. Once a command finishes -- indicated by this method
   * returning true -- the scheduler will call its {@link #end(boolean)} method.
   *
   * <p>Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always return
   * true will result in the command executing once and finishing immediately. It is recommended to
   * use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an
   * operation.
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally --
   * that is it is called when {@link #isFinished()} returns true -- or when it is
   * interrupted/canceled. This is where you may want to wrap up loose ends, like shutting off a
   * motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    ramseteCommand.end(interrupted);
  }
}

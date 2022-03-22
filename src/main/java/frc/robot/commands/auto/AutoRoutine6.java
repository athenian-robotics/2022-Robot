package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;


public class AutoRoutine6 extends CommandBase {
    private final RamseteCommand ramseteCommand;

    public AutoRoutine6(DrivetrainSubsystem drivetrainSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(drivetrainSubsystem);

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(
                        new SimpleMotorFeedforward(
                                Constants.AutoConstants.ksVolts,
                                Constants.AutoConstants.kvVoltSecondsPerMeter,
                                Constants.AutoConstants.kaVoltSecondsSquaredPerMeter),
                        Constants.AutoConstants.kDriveKinematics,
                        Constants.AutoConstants.maxVolts);

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.maxAutoSpeed,
                        Constants.AutoConstants.maxAutoAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.AutoConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        // create a new trajectory 1 meter forward
        Trajectory exampleTrajectory = PathPlanner.loadPath("New Path", 0.5, 0.5);

        this.ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                drivetrainSubsystem::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
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

        // Reset odometry to the starting pose of the trajectory.
        drivetrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
        // Run path following command, then stop at the end.
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        ramseteCommand.initialize();
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        ramseteCommand.execute();
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return ramseteCommand.isFinished();
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        ramseteCommand.end(interrupted);
    }
}

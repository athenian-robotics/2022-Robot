package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.auto.components.GoalNotFoundException;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


public class TurnToGoal extends CommandBase {
    DrivetrainSubsystem drivetrainSubsystem;
    LimelightSubsystem limelightSubsystem;
    private double distance;
    private double angleToGoal;
    private double goalMissingStartTime = 0;
    private double Kp = 0.01;
    private double Ki = 0.0;
    private double Kd = 0.0001;
    private double power = 0.0;
    private boolean goalFound = true;

    PIDController turnToGoalPID;

    public TurnToGoal(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        addRequirements(this.drivetrainSubsystem, this.limelightSubsystem);
    }

    @Override
    public void initialize() {
        try {
            //getLimelightOutputAtIndex() throws GoalNotFoundException
            distance = limelightSubsystem.getLimelightOutputAtIndex(0);
            angleToGoal = limelightSubsystem.getLimelightOutputAtIndex(1);

            //Scaling everything by our distance; farther = needs tighter tolerances
            Kp /= distance; Ki /= distance; Kd /= distance;
            turnToGoalPID = new PIDController(Kp, Ki, Kd);
            turnToGoalPID.setTolerance(5 / distance);
            turnToGoalPID.setSetpoint(angleToGoal);
        } catch (GoalNotFoundException e) {this.cancel();} //execute() should never run if the goal is not found initially
    }

    @Override
    public void execute() {
        try {
            power = turnToGoalPID.calculate(limelightSubsystem.getLimelightOutputAtIndex(1));
            drivetrainSubsystem.tankDrive(power, -power); updateAttendance(true);
        } catch (GoalNotFoundException e) {updateAttendance(false);}
    }

    @Override
    public boolean isFinished() {return turnToGoalPID.atSetpoint();}

    @Override
    public void end(boolean interrupted) {drivetrainSubsystem.tankDrive(0, 0);}

    //Helps avoid the command shutting down when losing limelight data only momentarily
    private void updateAttendance(boolean goalFound) {if (goalFound) {goalMissingStartTime = 0;} else if (goalMissingStartTime==0) {goalMissingStartTime = System.currentTimeMillis();} else if (System.currentTimeMillis() - goalMissingStartTime > 250) {this.cancel();}}
}

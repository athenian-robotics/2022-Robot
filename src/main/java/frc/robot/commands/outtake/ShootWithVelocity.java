package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.shooterData.InterpolatedTreeMap;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;

public class ShootWithVelocity extends CommandBase {
    private InterpolatedTreeMap veloData;
    private double targetVelocity;
    private OuttakeSubsystem outtake;
    private LimelightSubsystem limelight;

    public ShootWithVelocity(OuttakeSubsystem outtake, double targetVelocity, LimelightSubsystem limelight) {
        this.limelight = limelight;
        this.targetVelocity = targetVelocity;
        this.outtake = outtake;
        addRequirements(this.outtake);
        //  wheel rps to velocity
        veloData.put(0.0,0.0);
        veloData.put(1.0,0.0);
        veloData.put(2.0,0.0);
        // etc..
    }

    @Override
    public void execute() {
        if (limelight.isTargetFound()) {
            double distance = limelight.getDistance();
            double velocity =
                    Math.sqrt((9.81*distance*distance)/(2*(Constants.Shooter.hoodToHub + 1.73205080757*distance))); //
            double rps = veloData.get(velocity);
            //outtake.setRPS(rps);
        }
    }
}

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        addRequirements(this.outtakeSubsystem);
        //  wheel rpm to velocity
        veloData.put(0.0,0.0);
        veloData.put(1.0,0.0);
        veloData.put(2.0,0.0);
        // etc..
    }

    @Override
    public void execute() {
        if (limelight.isTargetFound()) {
            double distance = limelight.getDistance();
            double velocity = distance * 21321; // TODO: find acutal math
            double rpm = veloData.get(velocity);
            outtake.setShooterPower(rpm);
}
    }
}

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.maximumTurretAngleRadians;
import static frc.robot.Constants.MechanismConstants.minimumTurretAngleRadians;


public class TurretTurnToAngle extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    private final double angle;

    public TurretTurnToAngle(OuttakeSubsystem outtakeSubsystem, double angle) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.angle = angle;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void execute() {
        if (angle < minimumTurretAngleRadians && angle - outtakeSubsystem.getTurretAngleRadians() < 0 || angle > maximumTurretAngleRadians && angle - outtakeSubsystem.getTurretAngleRadians() > 0)
            System.out.println("Error! angle passed into TurretTurnToAngle " + this + " was out of bounds");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(outtakeSubsystem.getTurretAngleRadians() - Math.toRadians(angle)) <= 0.075; //0.5 position
        // tolerance
    }

    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.stopTurret();
    }
}

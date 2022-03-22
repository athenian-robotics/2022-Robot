package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.maximumTurretAngle;
import static frc.robot.Constants.MechanismConstants.minimumTurretAngle;


public class TurretTurnToAngle extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    private final double angle;

    public TurretTurnToAngle(OuttakeSubsystem outtakeSubsystem, double angle) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.angle = angle;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println(angle);
        if (angle < minimumTurretAngle && angle - outtakeSubsystem.getTurretAngle() < 0 || angle > maximumTurretAngle && angle - outtakeSubsystem.getTurretAngle() > 0)
            System.out.println("Error! angle passed into TurretTurnToAngle " + this + " was out of bounds");
        else
            outtakeSubsystem.turretAnglePID.setSetpoint(angle);
    }

    @Override
    public void execute() {
        outtakeSubsystem.turnTurret(outtakeSubsystem.turretAnglePID.calculate(outtakeSubsystem.getTurretAngle())); //lol
    }

    @Override
    public boolean isFinished() {
        return outtakeSubsystem.turretAnglePID.atSetpoint() && Math.abs(outtakeSubsystem.turretAnglePID.getVelocityError()) < 0.05; //0.5 position tolerance defined in OuttakeSubsystem
    }

    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.turnTurret(0);
        outtakeSubsystem.turretAnglePID.setSetpoint(0); //Back to minimizing limelight offset to goal
        outtakeSubsystem.turretAnglePID.reset();
    }
}

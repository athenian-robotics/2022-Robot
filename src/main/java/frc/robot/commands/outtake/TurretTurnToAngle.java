package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.OuttakeSubsystem;


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
        if (angle < Constants.MechanismConstants.minimumTurretAngle || angle > Constants.MechanismConstants.maximumTurretAngle) this.cancel();
        outtakeSubsystem.turretAnglePID.setSetpoint(angle);
    }

    @Override
    public void execute() {
        outtakeSubsystem.turnTurret(-outtakeSubsystem.turretAnglePID.calculate(outtakeSubsystem.getTurretPosition())); //lol
    }

    @Override
    public boolean isFinished() {
        return outtakeSubsystem.turretAnglePID.atSetpoint(); //0.5 tolerance defined in OuttakeSubsystem
    }

    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.turnTurret(0);
        outtakeSubsystem.turretAnglePID.setSetpoint(0); //Back to minimizing limelight offset to goal
    }
}

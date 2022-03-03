package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;


public class ManualAdjustTurret extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public ManualAdjustTurret(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (FightStick.fightStickJoystick.getX() < 0) { //TURRET ADJUSTMENT FALCON
            outtakeSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0) {
            outtakeSubsystem.turnTurret(turretTurnSpeed);
        } else {
            outtakeSubsystem.turnTurret(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    } //TODO CHANGE

    @Override
    public void end(boolean interrupted) {}
}

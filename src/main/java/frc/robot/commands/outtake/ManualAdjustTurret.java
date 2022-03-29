package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.slowTurretTurnSpeed;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;


//For use only in auto commands or as a default--you have to manually end!
public class ManualAdjustTurret extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public ManualAdjustTurret(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements();
    }

    @Override
    public void execute() {
        if (FightStick.fightStickJoystick.getX() < -0.5) { //TURRET ADJUSTMENT FALCON
            outtakeSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.5) {
            outtakeSubsystem.turnTurret(turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() < -0.5) { //TURRET ADJUSTMENT FALCON
            outtakeSubsystem.turnTurret(-slowTurretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() > 0.5) {
            outtakeSubsystem.turnTurret(slowTurretTurnSpeed);
        } else {
            outtakeSubsystem.turnTurret(0);
        }
    }
}

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.MechanismConstants.slowTurretTurnSpeed;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;


//For use only in auto commands or as a default--you have to manually end!
public class ManualAdjustTurret extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;

    public ManualAdjustTurret(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements();
    }

    @Override
    public void execute() {
        if (FightStick.fightStickJoystick.getX() < -0.5) { //TURRET ADJUSTMENT FALCON
            shooterSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.5) {
            shooterSubsystem.turnTurret(turretTurnSpeed);
        }else if (FightStick.fightStickJoystick.getY() < -0.5) { //TURRET ADJUSTMENT FALCON
                shooterSubsystem.turnTurret(-slowTurretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() > 0.5) {
                shooterSubsystem.turnTurret(slowTurretTurnSpeed);
            }
        else {
            shooterSubsystem.turnTurret(0);
        }
    }
}

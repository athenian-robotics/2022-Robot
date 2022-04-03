package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.TurretSubsystem;

import static frc.robot.Constants.MechanismConstants.slowTurretTurnSpeed;
import static frc.robot.Constants.MechanismConstants.turretTurnSpeed;


//You have to manually end this command!
public class ManualAdjustTurret extends CommandBase {
    private final TurretSubsystem turretSubsystem;

    public ManualAdjustTurret(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(this.turretSubsystem);
    }

    @Override
    public void execute() {
        if (FightStick.fightStickJoystick.getX() < -0.5) {
            turretSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.5) {
            turretSubsystem.turnTurret(turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() < -0.5) {
            turretSubsystem.turnTurret(-slowTurretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getY() > 0.5) {
            turretSubsystem.turnTurret(slowTurretTurnSpeed);
        } else {
            turretSubsystem.turnTurret(0);
        }
    }
}
package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.idleTurretSpeed;


public class ManualAdjustOuttake extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public ManualAdjustOuttake(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // TURRET ANGLE FALCON
        if (FightStick.fightStickJoystick.getX() < -0.5) {
            outtakeSubsystem.manualAdjustTurret(-idleTurretSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.5) {
            outtakeSubsystem.manualAdjustTurret(idleTurretSpeed);
        } else {
            outtakeSubsystem.manualAdjustTurret(0);
        }

        // HOOD ANGLE LINEAR SERVOS
        if (FightStick.fightStickJoystick.getY() < 0) { // Inverted
            outtakeSubsystem.manualAdjustHoodAngle(1);
        } else if (FightStick.fightStickJoystick.getY() > 0) {
            outtakeSubsystem.manualAdjustHoodAngle(-1);
        } else {

        }
    }

    @Override
    public boolean isFinished() {
        return false; // FOR DEFAULT, KEEP FALSE
    }

    @Override
    public void end(boolean interrupted) {

    }
}

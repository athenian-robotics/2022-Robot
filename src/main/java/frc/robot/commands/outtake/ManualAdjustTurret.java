package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.OuttakeSubsystem;


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
        if (FightStick.fightStickJoystick.getX() < 0) { // Inverted
            outtakeSubsystem.manualAdjustHoodAngle(1);
        } else if (FightStick.fightStickJoystick.getX() > 0) {
            outtakeSubsystem.manualAdjustHoodAngle(-1);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}

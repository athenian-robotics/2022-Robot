package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.MechanismConstants.*;


public class ManualAdjustOuttake extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;

    public ManualAdjustOuttake(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(this.shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // TURRET ANGLE FALCON
        if (FightStick.fightStickJoystick.getX() < -0.5) {
            shooterSubsystem.turnTurret(-turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.5) {
            shooterSubsystem.turnTurret(turretTurnSpeed);
        } else {
            shooterSubsystem.turnTurret(0);
        }

        // HOOD ANGLE LINEAR SERVOS
        if (FightStick.fightStickJoystick.getY() < 0) { // Inverted
            shooterSubsystem.setHoodAngle(maximumHoodAngle);
        } else if (FightStick.fightStickJoystick.getY() > 0) {
            shooterSubsystem.setHoodAngle(minimumHoodAngle);
        } else {
            shooterSubsystem.stopHood();
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

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.idleOuttakeSpeed;


public class EnableShooter extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public EnableShooter(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() { outtakeSubsystem.setShooterPower(idleOuttakeSpeed); }

    @Override
    public boolean isFinished() { return true; }
}

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class DisableShooter extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;

    public DisableShooter(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        outtakeSubsystem.setShooterPower(0.0);
        outtakeSubsystem.setHoodAngle(20);
    }

    @Override
    public boolean isFinished() { return true; }
}

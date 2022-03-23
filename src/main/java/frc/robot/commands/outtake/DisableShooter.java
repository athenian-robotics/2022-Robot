package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OuttakeSubsystem;


public class DisableShooter extends InstantCommand {
    private final OuttakeSubsystem outtakeSubsystem;

    public DisableShooter(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {
        outtakeSubsystem.stopShooter();
    }
}

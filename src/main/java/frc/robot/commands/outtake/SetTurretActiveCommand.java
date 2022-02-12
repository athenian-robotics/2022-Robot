package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OuttakeSubsystem;


public class SetTurretActiveCommand extends InstantCommand {
    private final OuttakeSubsystem outtakeSubsystem;
    private final boolean isActive;

    public SetTurretActiveCommand(OuttakeSubsystem outtakeSubsystem, boolean isActive) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
        this.isActive = isActive;
    }

    @Override
    public void initialize() {
        this.outtakeSubsystem.setTurretActive(isActive);
    }
}

package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OuttakeSubsystem;


public class SetShooterPower extends InstantCommand {
    private final OuttakeSubsystem outtakeSubsystem;
    private final double power;

    public SetShooterPower(OuttakeSubsystem outtakeSubsystem, double rps) {
        this.outtakeSubsystem = outtakeSubsystem; this.power = rps;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        outtakeSubsystem.setRPS(power);
    }
}

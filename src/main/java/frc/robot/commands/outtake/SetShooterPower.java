package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;


public class SetShooterPower extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    private final double power;

    public SetShooterPower(OuttakeSubsystem outtakeSubsystem, double rps) {
        this.outtakeSubsystem = outtakeSubsystem; this.power = rps;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void execute() {
        outtakeSubsystem.setRPS(power);
    }
}

package frc.robot.commands.outtake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.OuttakeSubsystem;


public class DisableShooter extends InstantCommand {
    private final OuttakeSubsystem outtakeSubsystem;

    public DisableShooter(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        outtakeSubsystem.stopShooter();
    }
}

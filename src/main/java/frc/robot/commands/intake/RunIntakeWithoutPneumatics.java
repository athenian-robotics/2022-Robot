package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;


public class RunIntakeWithoutPneumatics extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public RunIntakeWithoutPneumatics(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.startIntake();
        intakeSubsystem.startIntakeToIndexerMotor();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopIntakeToIndexerMotor();
    }
}

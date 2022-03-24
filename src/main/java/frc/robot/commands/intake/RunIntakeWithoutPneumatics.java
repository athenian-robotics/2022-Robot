package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


//MUST BE ENDED MANUALLY
public class RunIntakeWithoutPneumatics extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public RunIntakeWithoutPneumatics(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        if(!intakeSubsystem.isExtended) {
            intakeSubsystem.startIntake();
            intakeSubsystem.startIntakeToIndexerMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!intakeSubsystem.isExtended){
            intakeSubsystem.stopIntake();
            intakeSubsystem.stopIntakeToIndexerMotor();
        }
    }
}

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PortalSubsystem;


//MUST BE ENDED MANUALLY
public class RunIntakeWithoutPneumatics extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final PortalSubsystem portalSubsystem;

    public RunIntakeWithoutPneumatics(IntakeSubsystem intakeSubsystem, PortalSubsystem portalSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.portalSubsystem = portalSubsystem;
        addRequirements(this.intakeSubsystem, portalSubsystem);
    }

    @Override
    public void initialize() {
        if(!intakeSubsystem.isExtended) {
            intakeSubsystem.startIntake();
            portalSubsystem.startPortal();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!intakeSubsystem.isExtended){
            intakeSubsystem.stopIntake();
            portalSubsystem.stopPortal();
        }
    }
}

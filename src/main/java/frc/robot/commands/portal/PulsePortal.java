package frc.robot.commands.portal;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PortalSubsystem;


public class PulsePortal extends CommandBase {
    private final PortalSubsystem portalSubsystem;
    private final long milliseconds;
    private long start;

    public PulsePortal(PortalSubsystem portalSubsystem) {
        this.portalSubsystem = portalSubsystem;
        this.milliseconds = Long.MAX_VALUE;
        addRequirements(this.portalSubsystem);
    }

    public PulsePortal(PortalSubsystem portalSubsystem, double seconds) {
        this.portalSubsystem = portalSubsystem;
        this.milliseconds = (long) ((long) 1000 * seconds);
        addRequirements(this.portalSubsystem);
    }

    @Override
    public void initialize() {
        portalSubsystem.startPortal();
        start = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - start > milliseconds;
    }

    @Override
    public void end(boolean interrupted) {
        portalSubsystem.stopPortal();
    }
}

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;


public class PulseIntakeToIndexerMotor extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private long start;
    private final long milliseconds;

    public PulseIntakeToIndexerMotor(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.milliseconds = Long.MAX_VALUE;
        addRequirements(this.intakeSubsystem);
    }

    public PulseIntakeToIndexerMotor(IntakeSubsystem intakeSubsystem, double seconds) {
        this.intakeSubsystem = intakeSubsystem;
        this.milliseconds = (long) ((long) 1000*seconds);
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.startIntakeToIndexerMotor();
        start = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - start > milliseconds;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeToIndexerMotor();
    }
}

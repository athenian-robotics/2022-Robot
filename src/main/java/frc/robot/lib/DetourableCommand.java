package frc.robot.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;


//Adds the ability to sidestep within an order-sensitive command execution scenario by temporarily replacing the
// current command with another.
//Calling the detour() method will immediately start executing the secondary command and the original won't start
// executing again until the secondary command ends.
//DetourableCommands can detour() several times and can detour to other DetourableCommands. ඞ ඞ ඞ
//A DetourableCommand ends when the default Command ends while executing or when the secondary command ends and
// endAfterDetour is set to true.
//Classes that extend DetourableCommand will need to implement the standard Command methods with an underscore before
// their names instead of overriding the standard functions.
public abstract class DetourableCommand extends CommandBase {
    private final DetourableCommand m_detourableCommand;
    private Command m_detourCommand;
    private boolean m_endAfterDetour = false;
    private boolean m_detoured = false;

    public DetourableCommand() {
        this.m_detourableCommand = this;
    }

    @Override
    public final void initialize() {
        m_detourableCommand._initialize();
    }

    @Override
    public final void execute() {
        if (m_detoured) {
            m_detourCommand.execute();
        } else {
            m_detourableCommand._execute();
        }
    }

    @Override
    public final boolean isFinished() {
        if (m_detoured) {
            if (m_detourCommand.isFinished()) {
                m_detourCommand.end(false);
                m_detoured = false;
                return m_endAfterDetour;
            } else return false;
        } else {
            return m_detourableCommand._isFinished();
        }
    }

    @Override
    public final void end(boolean interrupted) {
        m_detourableCommand._end(false);
    }

    public void detour(Command detourCommand) {
        m_detourCommand = detourCommand;
        m_detourCommand.initialize();
        m_detoured = true;
    }

    public void detour(Command detourCommand, boolean endAfterDetour) {
        m_detourCommand = detourCommand;
        m_detourCommand.initialize();
        m_detoured = true;

        m_endAfterDetour = endAfterDetour;
    }

    public abstract void _initialize();

    public abstract void _execute();

    public abstract boolean _isFinished();

    public abstract void _end(boolean interrupted);
}

//<3 Jacob '22
//but only if y'all use this lol
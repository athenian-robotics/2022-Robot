package frc.robot.commands.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;

import static frc.robot.Constants.MechanismConstants.DefaultHoodAngle;


public class DefaultAngleHoodShooter extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    PIDController pid = new PIDController(Kp, Ki, Kd);

    public DefaultAngleHoodShooter(OuttakeSubsystem outtakeSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(this.outtakeSubsystem);
    }

    @Override
    public void initialize() {
        pid.setSetpoint(DefaultHoodAngle);
    }

    @Override
    public void execute() {
        double power = pid.calculate(DefaultHoodAngle);
        outtakeSubsystem.moveHoodMotor(power);

    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

    }
}

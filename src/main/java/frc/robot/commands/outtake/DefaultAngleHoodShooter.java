package frc.robot.commands.outtake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuttakeSubsystem;

import frc.robot.Constants.MechanismConstants;


public class DefaultAngleHoodShooter extends CommandBase {
    private final OuttakeSubsystem outtake;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    PIDController pid = new PIDController(Kp, Ki, Kd);

    public DefaultAngleHoodShooter(OuttakeSubsystem outtake) {
        this.outtake = outtake;
        addRequirements(this.outtake);

        pid.setTolerance(1);
    }

    @Override
    public void initialize() {
        pid.setSetpoint(MechanismConstants.defaultHoodAngle);
    }

    @Override
    public void execute() {
        double power = pid.calculate(outtake.getHoodAngle());
        outtake.moveHoodMotor(power);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

    }
}

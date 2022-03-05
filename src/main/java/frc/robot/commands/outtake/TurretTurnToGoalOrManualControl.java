package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.GoalNotFoundException;
import frc.robot.lib.controllers.FightStick;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;


public class TurretTurnToGoalOrManualControl extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    private final LimelightSubsystem limelightSubsystem;

    public TurretTurnToGoalOrManualControl(OuttakeSubsystem outtakeSubsystem, LimelightSubsystem limelightSubsystem) {
        this.outtakeSubsystem = outtakeSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(this.outtakeSubsystem, this.limelightSubsystem);
    }

    @Override
    public void execute() {
        if (FightStick.fightStickJoystick.getX() < -0.75) {
            outtakeSubsystem.turnTurret(-Constants.MechanismConstants.turretTurnSpeed);
        } else if (FightStick.fightStickJoystick.getX() > 0.75) {
            outtakeSubsystem.turnTurret(Constants.MechanismConstants.turretTurnSpeed);
        } else if (limelightSubsystem.isTargetFound()) {
            try {
                outtakeSubsystem.turnTurret(-outtakeSubsystem.turretAnglePID.calculate(limelightSubsystem.getLimelightOutputAtIndex(1)));
            } catch (GoalNotFoundException ignored) {}
        } else {
            outtakeSubsystem.turnTurret(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtakeSubsystem.turnTurret(0);
    }
}

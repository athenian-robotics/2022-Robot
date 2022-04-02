package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static com.ctre.phoenix.motorcontrol.TalonFXControlMode.PercentOutput;
import static frc.robot.Constants.MechanismConstants.*;


public class TurretSubsystem extends SubsystemBase {
    public final WPI_TalonFX turretMotor = new WPI_TalonFX(turretMotorPort);
    public final SimpleMotorFeedforward feed;
    public final ProfiledPIDController turretPID =
            new ProfiledPIDController(3.1856, 0, 1.13513, new TrapezoidProfile.Constraints(Math.PI / 2, Math.PI / 2)); //

    public boolean turretRunning = false;
    public boolean bangBangRunning = false;

    public double currentShooterToleranceDegrees = 1;
    private double bangBangSetpointRadians;


    public TurretSubsystem() {
        turretMotor.setInverted(false);
        turretMotor.setNeutralMode(NeutralMode.Brake);

        this.feed = new SimpleMotorFeedforward(Constants.Turret.ks, Constants.Turret.kv, Constants.Turret.ka);
        setTurretStartingAngleDegrees(-180); //assume default position is turret starting facing backwards counterclockwise
    }

    public void turnTurret(double power) {
        if (power == 0.0) {
            disable();
        } else {
            turretMotor.set(ControlMode.PercentOutput, power > turretTurnSpeed ? turretTurnSpeed : Math.max(power, -turretTurnSpeed));
        }
    }

    public void turnTurretWithVoltage(double voltage) {
        if (voltage == 0.0) {
            disable();
        } else {
            turretMotor.setVoltage(voltage);
        }
    }

    public void setTurretStartingAngleDegrees(double position) {
        //Primarily for use in auto routines where we need to know where the shooter starts
        turretMotor.setSelectedSensorPosition(2048 * position / 36);
    }

    //CW Positive
    public void setTurretPositionRadians(double angle) {
        bangBangSetpointRadians = angle;
        bangBangRunning = true;
    }

    public double getTurretAngleRadians() {
        return Math.toRadians(turretMotor.getSelectedSensorPosition() * 36 / 2048);
    }

    public void disable() {
        turretMotor.set(PercentOutput, 0);
        turretRunning = false;
        bangBangRunning = false;
    }

    @Override
    public void periodic() {
        if (bangBangRunning) {
            double bangBangOffset = bangBangSetpointRadians - getTurretAngleRadians();
            if (Math.abs(bangBangOffset) >= currentShooterToleranceDegrees) {
                turnTurret(Math.signum(bangBangOffset) * slowTurretTurnSpeed + turretTurnSpeed / bangBangOffset);
            } else {
                disable();
            }
        }
    }
}


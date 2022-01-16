package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MechanismConstants.IntakeMotorPort;
import static frc.robot.Constants.PneumaticConstants.IntakePneumaticPort1;
import static frc.robot.Constants.PneumaticConstants.IntakePneumaticPort2;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(IntakeMotorPort);
    private final Solenoid intakePneumatic1 = new Solenoid(PneumaticsModuleType.REVPH,IntakePneumaticPort1);
    private final Solenoid intakePneumatic2 = new Solenoid(PneumaticsModuleType.REVPH,IntakePneumaticPort2);
    public boolean isRunning = false;
    public boolean isExtended = false;

    public IntakeSubsystem() {
    }

    public void startIntake() {
        intakeMotor.set(ControlMode.PercentOutput, 0.5);
        System.out.println("start intake");
        isRunning = true;
    }

    public void invert() {
        intakeMotor.setInverted(!intakeMotor.getInverted());
    }

    public void stopIntake() {
        intakeMotor.set(ControlMode.PercentOutput,0);
        System.out.println("stop intake");
        isRunning = false;
    }

    public void toggleIntake() {
        if (isRunning) {
            stopIntake();
        } else {
            startIntake();
        }
    }

    //push out intake over bumper
    public void extendIntakePneumatic(){
        intakePneumatic1.set(true);
        intakePneumatic2.set(true);
        isExtended=true;
    }

    //retract intake over bumper
    public void retractIntakePneumatic(){
        intakePneumatic1.set(false);
        intakePneumatic2.set(false);
        isExtended=false;
    }


    public void togglePneumatic(){
        if(isExtended){
            retractIntakePneumatic();
        }
        else{
            extendIntakePneumatic();
        }
    }

    @Override
    public void periodic(){}
}


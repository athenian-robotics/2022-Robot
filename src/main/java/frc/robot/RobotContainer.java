// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//aarav was here            balls

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.*;
import frc.robot.commands.climb.*;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.indexer.PulseIndexer;
import frc.robot.commands.indexer.QueueBalls;
import frc.robot.commands.intake.RunIntakeWithoutPneumatics;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.outtake.AlwaysTurretTurnToGoalWithLimelight;
import frc.robot.commands.outtake.DisableShooter;
import frc.robot.commands.outtake.EnableShooter;
import frc.robot.commands.outtake.ShootTwo;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.function.BooleanSupplier;

import static frc.robot.Constants.MechanismConstants.telescopeSpeed;
import static frc.robot.Constants.MechanismConstants.winchSpeed;


public class RobotContainer {
    // CONTROLLERS
    public static JoystickButton xboxA;
    public static JoystickButton xboxB;
    public static JoystickButton xboxX;
    public static JoystickButton xboxY;
    public static JoystickButton xboxLB;
    public static JoystickButton xboxRB;
    public static JoystickButton xboxLP;
    public static JoystickButton xboxRP;
    public static JoystickButton xboxSquares;
    public static JoystickButton xboxHamburger;
    public static Trigger xboxLS;
    public static XboxController.Axis xboxRS;
    public static XboxController xboxController = new XboxController(Constants.OIConstants.xboxControllerPort);
    SendableChooser<SequentialCommandGroup> chooser = new SendableChooser<>();
    // SUBSYSTEMS
    public static LimelightSubsystem limelight = new LimelightSubsystem("limelight-arc");
    public static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    public static IndexerSubsystem indexer = new IndexerSubsystem();
    public static IntakeSubsystem intake = new IntakeSubsystem();
    public static ClimberSubsystem climb = new ClimberSubsystem();
    public static OuttakeSubsystem outtake = new OuttakeSubsystem();
    //MISC
    public static DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
    public static ShooterDataTable shooterDataTable;

    // Sets up controllers, configures controllers, and sets the default drive mode (tank or arcade)
    public RobotContainer() {
        drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, xboxController)); // Check for Arcade or Tank
        //outtake.setDefaultCommand(new AlwaysTurretTurnToGoalWithLimelightOrManualControl(limelight, outtake)); // Check fight stick y-axis for manual hood adjustment
        indexer.setDefaultCommand(new QueueBalls(indexer, intake)); //Turns on indexer when sees a ball, sets it to
        // off
        // when there are no balls in sight

        try {
            ObjectInputStream fin = new ObjectInputStream(new FileInputStream("/home/lvuser/deploy/dt.ser"));
            Object obj = fin.readObject();
            if (obj instanceof ShooterDataTable) {
                shooterDataTable = (ShooterDataTable) obj;
                System.out.println("Checking 2.5m data in shooterDataTable: " + shooterDataTable.getSpecs(2.5));
            }
        } catch (Exception e) {
            System.out.println("file not found, or class not found");
        }

        SmartDashboard.putData("AutoChooser", chooser);
        chooser.setDefaultOption("0: 2.5 Meters Forward", new AutoRoutine0(drivetrain));
        //chooser.addOption("1: 5 Ball Auto - Bottom Left Start", new AutoRoutine1(climb, drivetrain, indexer, intake, outtake, limelight, shooterDataTable));
        //chooser.addOption("2: 3 Ball Auto - Top Left Start", new AutoRoutine2(climb, drivetrain, indexer, intake, outtake, limelight, shooterDataTable));
        //chooser.addOption("3: 2 Ball Auto - Bottom Left Start", new AutoRoutine3(climb, drivetrain, indexer, intake, outtake, limelight, shooterDataTable));

        xboxButtonSetup();
        configureButtonBindings();
    }

  // Configures xbox buttons to commands
  private void configureButtonBindings() {
      /*  SUBSYSTEM COMMANDS (Main, functional commands) */
      xboxHamburger.whenPressed(new ShootBalls(climb, drivetrain, indexer, intake, outtake, limelight, shooterDataTable));
      //xboxA.whenPressed(new TurretTurnToAngle(outtake, -90));
      FightStick.fightStickA.whenPressed(new ToggleIntake(intake)); // Toggle intake wheels and pneumatics
      xboxX.whenPressed(new ShootTwo(climb, drivetrain, indexer, intake, outtake, limelight, shooterDataTable));
      FightStick.fightStickL3.whenHeld(new WinchSetSpeed(climb, winchSpeed)); // Toggle indexer (tower portion)
      FightStick.fightStickR3.whenHeld(new WinchSetSpeed(climb, -winchSpeed)); //Toggle Indexer down (tower portion)
      //FightStick.fightStickB.whenPressed(new EnableShooter(outtake)); // Enable shooter wheels
      //FightStick.fightStickY.whenPressed(new DisableShooter(outtake)); // Disable shooter wheels
      FightStick.fightStickLB.whenHeld(new SetBothTelescopeSpeed(climb, -telescopeSpeed));
      FightStick.fightStickRB.whenHeld(new SetBothTelescopeSpeed(climb, telescopeSpeed));
      //FightStick.fightStickLB.whenPressed(new SetBothTelescopePositions(climb, 0));
      //FightStick.fightStickRB.whenPressed(new SetBothTelescopePositions(climb, 1));
      //FightStick.fightStickLT.whileActiveOnce(new LeftTelescopeSetSpeed(climb, -telescopeSpeed));
      //FightStick.fightStickRB.whenHeld(new RunIntakeWithoutPneumatics(intake, indexer));
      //FightStick.fightStickRT.whileActiveOnce(new RightTelescopeSetSpeed(climb, -telescopeSpeed));
      //FightStick.fightStickX.whenHeld(new RightTelescopeSetSpeed(climb, -0.2));
      xboxB.whenHeld(new RunIntakeWithoutPneumatics(intake, indexer));
      xboxY.whenPressed(new AlwaysTurretTurnToGoalWithLimelight(limelight, outtake));
      FightStick.fightStickX.whenPressed(new Traverse(climb));

        /* MISC COMMANDS (Random lib of commands. Written using functional commands because most are just one line ) */
        // have fun with this - jason and jacob '22   ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ
        xboxSquares.whenPressed(new FunctionalCommand(  // Toggle drive mode
                () -> {
                    if (drivetrain.getDefaultCommand() instanceof ArcadeDrive)
                        drivetrain.setDefaultCommand(new TankDrive(drivetrain, xboxController));
                    else drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, xboxController));
                }, () -> {
        }, interrupted -> {
        }, () -> true, drivetrain));
        xboxLP.whenPressed(new FunctionalCommand(() -> drivetrain.shiftDown(), () -> {
        }, interrupted -> {
        }, () -> true, drivetrain)); // Shift down
        xboxRP.whenPressed(new FunctionalCommand(() -> drivetrain.shiftUp(), () -> {
        }, interrupted -> {
        }, () -> true, drivetrain)); // Shift up
    }

    // Connects xbox buttons to button #'s for the driver station
    private void xboxButtonSetup() {
        xboxA = new JoystickButton(xboxController, 1);
        xboxB = new JoystickButton(xboxController, 2);
        xboxX = new JoystickButton(xboxController, 3);
        xboxY = new JoystickButton(xboxController, 4);
        xboxLB = new JoystickButton(xboxController, 5);
        xboxRB = new JoystickButton(xboxController, 6);
        xboxLP = new JoystickButton(xboxController, 9);
        xboxRP = new JoystickButton(xboxController, 10);
        xboxSquares = new JoystickButton(xboxController, 7);
        xboxHamburger = new JoystickButton(xboxController, 8);
        xboxLS = new Trigger();
    }

    // Disables all robot subsystems (Emergency only)
    public void disableAll() {
        drivetrain.disable();
        indexer.disable();
        //intake.disable();
        limelight.disable();
        outtake.disable();
    }

    public void setAlliance(DriverStation.Alliance alliance) {if (alliance != DriverStation.Alliance.Invalid) RobotContainer.alliance = alliance;}

    // Returns the robot's main autonomous command
    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}


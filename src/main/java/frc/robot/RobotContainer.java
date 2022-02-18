// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.AutoRoutine6;
import frc.robot.commands.climb.LeftTelescopeSetSpeed;
import frc.robot.commands.climb.RightTelescopeSetSpeed;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.commands.indexer.PulseIndexer;
import frc.robot.commands.indexer.QueueBalls;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.outtake.DisableShooter;
import frc.robot.commands.outtake.EnableShooter;
import frc.robot.commands.outtake.ManualAdjustHoodAngle;
import frc.robot.commands.outtake.ShootOneBall;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;

import java.io.FileInputStream;
import java.io.ObjectInputStream;

import static frc.robot.Constants.MechanismConstants.telescopeSpeed;


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
    // SUBSYSTEMS
    public static DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    public static IndexerSubsystem indexer = new IndexerSubsystem();
    public static IntakeSubsystem intake = new IntakeSubsystem();
    public static ClimberSubsystem climb = new ClimberSubsystem();
    public static LimelightSubsystem limelight = new LimelightSubsystem("limelight-arc");
    public static OuttakeSubsystem outtake = new OuttakeSubsystem(limelight);
    public static ShooterDataTable shooterDataTable = new ShooterDataTable();

    // Sets up controllers, configures controllers, and sets the default drive mode (tank or arcade)
    public RobotContainer() {
        xboxButtonSetup();
        configureButtonBindings();

        drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, xboxController)); // Check for Arcade or Tank
        outtake.setDefaultCommand(new ManualAdjustHoodAngle(outtake)); // Check fight stick y-axis for manual hood adjustment
        indexer.setDefaultCommand(new QueueBalls(indexer)); //Turns on indexer when sees a ball, sets it to off when there are no balls in sight
        try{
            ObjectInputStream fin = new ObjectInputStream(new FileInputStream(    "/home/lvuser/deploy/dt.ser"));
            Object obj = fin.readObject();
            if(obj instanceof ShooterDataTable) {
                shooterDataTable = (ShooterDataTable) obj;
                System.out.println("Checking 1.1m data in shooterDataTable: " + shooterDataTable.getSpecs(1.1));
            }
        }catch(Exception e){
            System.out.println("file not found, or class not found");
        }
    }

  // Configures xbox buttons to commands
  private void configureButtonBindings() {
      /*  SUBSYSTEM COMMANDS (Main, functional commands) */
      xboxHamburger.whenPressed(new ShootOneBall(drivetrain, indexer, intake, limelight, outtake, shooterDataTable));
      FightStick.fightStickA.whenPressed(new ToggleIntake(intake)); // Toggle intake wheels and pneumatics
      // FightStick.fightStickX.whenPressed(new ToggleIndexer(indexer));
      FightStick.fightStickL3.whenHeld(new PulseIndexer(indexer, true)); // Toggle indexer (tower portion)
      FightStick.fightStickB.whenPressed(new EnableShooter(outtake)); // Enable shooter wheels
      FightStick.fightStickY.whenPressed(new DisableShooter(outtake)); // Disable shooter wheels
      FightStick.fightStickLB.whenHeld(new LeftTelescopeSetSpeed(climb, telescopeSpeed)); // Left Cots Climb Up
      FightStick.fightStickLT.whenActive(new LeftTelescopeSetSpeed(climb, -telescopeSpeed)); //Left Cots Climb Down
      FightStick.fightStickRB.whenHeld(new RightTelescopeSetSpeed(climb, telescopeSpeed)); // Right Cots Climb Up
      FightStick.fightStickRT.whenActive(new RightTelescopeSetSpeed(climb, -telescopeSpeed)); // Right Cots Climb Down

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
        intake.disable();
        limelight.disable();
        outtake.disable();
    }

    // Returns the robot's main autonomous command
    public Command getAutonomousCommand() {
        return new AutoRoutine6(drivetrain);
    }
}


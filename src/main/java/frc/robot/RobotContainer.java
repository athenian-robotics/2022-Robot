// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.Forward2AndHalfMeters;
import frc.robot.auto.TwoBallBottomLeft;
import frc.robot.auto.TwoBallMidBottom;
import frc.robot.auto.TwoBallTopLeftDefense;
import frc.robot.lib.controllers.FightStick;
import frc.robot.lib.shooterData.ShooterDataTable;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Logger;
import java.io.FileInputStream;
import java.io.ObjectInputStream;

public class RobotContainer {
  public static ShooterDataTable shooterDataTable;
  public static final XboxController xboxController =
      new XboxController(Constants.OIConstants.xboxControllerPort);
  // SUBSYSTEMS

  public static final LimelightSubsystem limelight = new LimelightSubsystem();
  public static PoseEstimator poseEstimator;
  public static final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public static final IndexerSubsystem indexer = new IndexerSubsystem();
  public static final PortalSubsystem portal = new PortalSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  public static final ClimberSubsystem climb = new ClimberSubsystem();
  public static ShooterSubsystem shooter;
  public static HoodSubsystem hood;
  public static Superstructure superstructure;
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
  public static Axis xboxRS;
  // MISC
  public static DriverStation.Alliance alliance = DriverStation.getAlliance();
  final SendableChooser<SequentialCommandGroup> chooser = new SendableChooser<>();

  // Sets up controllers, configures controllers, and sets the default drive mode (tank or arcade)
  public RobotContainer() {
    try {
      ObjectInputStream fin =
          new ObjectInputStream(new FileInputStream("/home/lvuser/deploy/dt.ser"));
      Object obj = fin.readObject();
      if (obj instanceof ShooterDataTable) {
        shooterDataTable = (ShooterDataTable) obj;
        System.out.println(
            "Checking 2.5m data in shooterDataTable: " + shooterDataTable.getSpecs(2.5));
      }
    } catch (Exception e) {
      System.out.println("file not found, or class not found: " + e);
    }
    poseEstimator = new PoseEstimator(shooterDataTable);
    shooter = new ShooterSubsystem(poseEstimator, shooterDataTable);
    hood = new HoodSubsystem(shooterDataTable, limelight);
    superstructure = new Superstructure(hood, portal, shooter, indexer);
    xboxButtonSetup();
    configureButtonBindings();
    configureAutoChooser();
    configureDefaultCommands();
    portForwardLimelightPorts();

    Shuffleboard.getTab("852 - Dashboard")
        .add("Chooser", chooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
    Logger.configureLoggingAndConfig(this, false);
  }

  // Configures xbox buttons to commands
  private void configureButtonBindings() {
    FightStick.fightStickA.whenPressed(intake.suckExtended());

    // FightStick.fightStickY.whenPressed(superstructure.shoot());
    FightStick.fightStickB.whenPressed(intake.suckRetracted());
    FightStick.fightStickLB.whenHeld(climb.telescopeDown());
    FightStick.fightStickRB.whenHeld(climb.telescopeUp());
    FightStick.fightStickOption.whenPressed(hood.approachTarget());
    FightStick.fightStickY.whenPressed(superstructure.shootHub());
    //    fightStickLT.whenActive(
    //        new ShootLowGoalNextToTarget(drivetrain, indexer, intake, shooter, hood, portal,
    // turret));
    FightStick.fightStickX.whenPressed(intake.idleRetracted());
    // xboxB.whenPressed(superstructure.shootHub());
    //    xboxB.whenPressed(
    //        new ShootLowGoalNextToTarget(drivetrain, indexer, intake, shooter, hood, portal,
    // turret));
    xboxA.whenPressed(intake.suckExtended());
    xboxX.whenPressed(intake.idleRetracted());
    xboxRB.whenPressed(superstructure.shoot());

    /* MISC COMMANDS (Random lib of commands. Written using functional commands because most are just one line ) */
    // have fun with this - jason and jacob '22   ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ ඞ
    // two things: the amonguses broke CI and i had to fix and u wrote it wrong,
    // u should have used instant commands instead of functional commands rohan '24
    //    xboxSquares.whenPressed(
    //        new InstantCommand( // Toggle drive mode
    //            () -> {
    //              if (drivetrain.getDefaultCommand() instanceof ArcadeDrive)
    //                drivetrain.setDefaultCommand(
    //                    new RunCommand(
    //                        () ->
    //                            drivetrain.tankDrive(
    //                                -xboxController.getLeftY(), -xboxController.getRightY()),
    //                        drivetrain));
    //              else drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, xboxController));
    //            },
    //            drivetrain));
    xboxLP.whenPressed(new InstantCommand(drivetrain::shiftDown, drivetrain)); // Shift down
    xboxRP.whenPressed(new InstantCommand(drivetrain::shiftUp, drivetrain)); // Shift up
    xboxB.whenPressed(intake.suckRetracted());
    xboxY.whenPressed(superstructure.shootHub());
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

  private void configureAutoChooser() {
    chooser.setDefaultOption(
        "2.5 Meters Forward", new Forward2AndHalfMeters(drivetrain, poseEstimator));
    chooser.addOption(
        "TWoBallBottomLeft",
        new TwoBallBottomLeft(drivetrain, intake, superstructure, poseEstimator));
    chooser.addOption(
        "TwoBallMidBottom",
        new TwoBallMidBottom(drivetrain, intake, poseEstimator, superstructure));
    chooser.addOption(
        "twoBallTopLeftDefense",
        new TwoBallTopLeftDefense(drivetrain, intake, poseEstimator, superstructure));
    //    chooser.setDefaultOption("0: 2.5 Meters Forward", new Forward2AndHalfMeters(drivetrain));
    //    chooser.addOption(
    //        "1: 5 Ball Auto - Bottom Left Start",
    //        new FiveBallBottomLeft(
    //            drivetrain,
    //            indexer,
    //            intake,
    //            shooter,
    //            turret,
    //            hood,
    //            portal,
    //            limelight,
    //            shooterDataTable));
    //    chooser.addOption(
    //        "2: 2 Ball Auto - Top Left Start",
    //        new TwoBallTopLeft(
    //            drivetrain,
    //            indexer,
    //            intake,
    //            shooter,
    //            turret,
    //            hood,
    //            portal,
    //            limelight,
    //            shooterDataTable));
    //    chooser.addOption(
    //        "3: 2 Ball Auto - Bottom Left Start",
    //        new TwoBallBottomLeft(
    //            drivetrain,
    //            indexer,
    //            intake,
    //            shooter,
    //            turret,
    //            hood,
    //            portal,
    //            limelight,
    //            shooterDataTable));
    //    chooser.addOption(
    //        "4: 4 Ball Auto - Bottom Left Start",
    //        new FourBallBottomLeft(
    //            drivetrain,
    //            indexer,
    //            intake,
    //            shooter,
    //            turret,
    //            hood,
    //            portal,
    //            limelight,
    //            shooterDataTable));
    //    chooser.addOption(
    //        "5: 2 ball Auto Mid Bottom",
    //        new TwoBallMidBottom(
    //            drivetrain,
    //            indexer,
    //            intake,
    //            shooter,
    //            turret,
    //            hood,
    //            portal,
    //            limelight,
    //            shooterDataTable));
    //    chooser.addOption(
    //        "7: 2 Ball Auto - Top Left Start And Defense",
    //        new TwoBallTopLeftDefense(
    //            drivetrain,
    //            indexer,
    //            intake,
    //            shooter,
    //            turret,
    //            hood,
    //            portal,
    //            limelight,
    //            shooterDataTable));
  }

  public void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.curvatureDrive(-xboxController.getLeftY(), xboxController.getRightX()),
            drivetrain)); //
  }

  public void setAlliance(DriverStation.Alliance alliance) {
    if (alliance != DriverStation.Alliance.Invalid) RobotContainer.alliance = alliance;
  }

  // Returns the robot's main autonomous command
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public void portForwardLimelightPorts() {
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);
    PortForwarder.add(5800, "10.8.52.76", 5800);
    PortForwarder.add(5801, "10.8.52.76", 5801);
    PortForwarder.add(5802, "10.8.52.76", 5802);
    PortForwarder.add(5803, "10.8.52.76", 5803);
    PortForwarder.add(5804, "10.8.52.76", 5804);
    PortForwarder.add(5805, "10.8.52.76", 5805);
  }

  // Disables all robot subsystems
  public void disableAll() {
    drivetrain.disable();
    hood.disable();
    indexer.disable();
    portal.disable();
  }
}

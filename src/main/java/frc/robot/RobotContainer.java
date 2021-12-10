// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final ExampleCommand m_autoCommand = new ExampleCommand();

  public RobotContainer() {

    configureButtonBindings();
  }

  private void configureButtonBindings() {}


  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}

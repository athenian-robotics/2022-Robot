package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ExampleAuto extends CommandBase {

  public ExampleAuto(DrivetrainSubsystem drivetrainSubsystem) {
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}

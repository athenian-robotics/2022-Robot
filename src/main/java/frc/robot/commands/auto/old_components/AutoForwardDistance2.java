package frc.robot.commands.auto.old_components;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoForwardDistance2 extends CommandBase {
  private final double setpoint;
  private final Encoder leftEnc;
  private final Encoder rightEnc;
  private final DrivetrainSubsystem drivetrainSubsystem;

  public AutoForwardDistance2(DrivetrainSubsystem drivetrainSubsystem, double metersToDrive) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    leftEnc = drivetrainSubsystem.leftEncoder;
    rightEnc = drivetrainSubsystem.rightEncoder;
    setpoint = ((leftEnc.getDistance() + rightEnc.getDistance()) / 2) + metersToDrive;
    addRequirements(this.drivetrainSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrainSubsystem.tankDrive(0.3, 0.3);
  }

  @Override
  public boolean isFinished() {
    return ((leftEnc.getDistance() + rightEnc.getDistance()) / 2 >= setpoint + 0.01);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.tankDrive(0, 0);
  }
}

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.limelight.GoalNotFoundException;
import frc.robot.lib.limelight.LimelightDataLatch;
import frc.robot.lib.limelight.LimelightDataType;
import frc.robot.subsystems.LimelightSubsystem;

public class GuaranteeLimelightData extends CommandBase {
  private final LimelightSubsystem limelightSubsystem;
  private final LimelightDataLatch latch;

  public GuaranteeLimelightData(LimelightSubsystem limelightSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    latch = new LimelightDataLatch(LimelightDataType.DISTANCE, 5);
  }

  @Override
  public void initialize() {
    limelightSubsystem.addLatch(latch);
  }

  @Override
  public boolean isFinished() {
    try {
      return latch.unlocked(); // the command ends when we retrieve valid limelight data
    } catch (GoalNotFoundException e) {
      limelightSubsystem.addLatch(latch.reset());
      return false; // timeout protection
    }
  }
}

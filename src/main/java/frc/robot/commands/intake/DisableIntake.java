package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;


public class DisableIntake extends InstantCommand {private final IntakeSubsystem intakeSubsystem; public DisableIntake(IntakeSubsystem intakeSubsystem) {this.intakeSubsystem = intakeSubsystem; addRequirements(this.intakeSubsystem);}
    @Override public void initialize() {intakeSubsystem.disable();}}
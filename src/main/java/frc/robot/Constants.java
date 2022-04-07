// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final double looptime = 0.02;

  public static final class AutoConstants {
    public static final double ksVolts = 0.55834;
    public static final double kvVoltSecondsPerMeter = 3.3207;
    public static final double kaVoltSecondsSquaredPerMeter = 0.51012;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(DriveConstants.trackWidth);
    public static final double maxVolts = 7.0;
    public static final double maxAutoSpeed = 4; // meters per second 3
    public static final double maxAutoAcceleration = 1.5; // meters per second per second  1.5
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;
    public static final double kPDriveVel = 4.5729;
    public static final double maxAutoTurn = 0.55;
  }

  public static final class DriveConstants {
    /** DRIVE MOTOR PORTS */
    public static final int leftFrontDrivePort = 2;

    public static final int leftRearDrivePort = 1;
    public static final int rightFrontDrivePort = 3;
    public static final int rightRearDrivePort = 4;

    public static final int rightEncoderChannelA = 2;
    public static final int rightEncoderChannelB = 3;
    public static final int leftEncoderChannelA = 0;
    public static final int leftEncoderChannelB = 1;

    /** MISC CONSTANTS */
    public static final double trackWidth = Units.inchesToMeters(28);

    public static final double driveGearRatio = 1;
    public static final double wheelDiameter = 6.0; // 6 in.

    public static final double minDriveSpeed = 0.1;
    public static final double maxDriveSpeed = 0.95;
    public static final double maxAutoSpeed = 0.55;
  }

  public static final class OIConstants {
    /** XBOX CONTROLLER PORTS */
    public static final int xboxControllerPort = 1;

    public static final int fightStickPort = 0;
  }

  public static final class MechanismConstants {
    // INDEXER
    public static final int intakeMotorPort = 6;
    public static final int indexerMotorPort = 12;
    public static final int intakeToIndexerMotorPort = 13;
    public static final double indexerSpeed = 0.35; // 0.25
    public static final double intakeToIndexerSpeed = 0.2;
    public static final int intakeToIndexerResidualIndexTimeMillis = 700;
    // CLIMBER
    public static final int leftClimberMotorPort = 15;
    public static final int rightClimberMotorPort = 16;
    public static final int climbWinchMotorPort = 17;
    public static final double telescopeSpeed = 0.5;
    public static final double winchSpeed = 0.25;
    // TURRET
    public static final int turretMotorPort = 10;
    public static final double turretTurnSpeed = 0.075;
    public static final double slowTurretTurnSpeed = 0.06;
    // SHOOTER
    public static final int shooterMotorPortA = 7;
    public static final int shooterMotorPortB = 8;
    // SHOOTER HOOD
    public static final int leftHoodServoPort = 2;
    public static final int rightHoodServoPort = 3;
    public static final double minimumHoodAngle = 8;
    public static final double maximumHoodAngle = 41;
    public static final double defaultHoodAngle = 31;
    public static final double maximumTurretAngleRadians = Math.toRadians(90);
    public static final double minimumTurretAngleRadians = Math.toRadians(-210);
    // MAX MECHANISM SPEEDS
    public static final double intakeSpeed = 0.3;
  }

  public static final class PneumaticConstants {
    public static final int shifterRightSolenoidPortA = 1; // SOLENOID 2
    public static final int shifterRightSolenoidPortB = 4;
    public static final int shifterLeftSolenoidPortA = 7; // SOLENOID 1
    public static final int shifterLeftSolenoidPortB = 3;
    public static final int pneumaticPortRightA = 6; // SOLENOID 4
    public static final int pneumaticPortRightB = 5;
    public static final int pneumaticPortLeftA = 0; // SOLENOID 3
    public static final int pneumaticPortLeftB = 2;

    // Pneumatics constants ...
  }

  public static final class LEDConstants {
    public static final int LEDPort = 0;
    public static final int LEDStripLength = 40;
  }

  public static final class Shooter {
    public static final double ks = 0.51149;
    public static final double ka = 0.0091765;
    public static final double kv = 0.10818;

    public static final double maxError = 0.05;
    public static final double maxControlEffort = 12;
    public static final double modelDeviation = 0.075;
    public static final double encoderDeviation = 0.02;
  }

  public static final class Turret {
    public static final double ks = 0.70514;
    public static final double ka = 0.019842;
    public static final double kv = 0.22251;
  }
}

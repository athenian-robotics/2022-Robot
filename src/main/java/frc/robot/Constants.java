// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double looptime = 0.02;
    public static final class AutoConstants {
        public static double ksVolts = 0.2455; //0.49117
        public static double kvVoltSecondsPerMeter = 1.3023; //3.4251
        public static double kaVoltSecondsSquaredPerMeter = 0.091565; //0.0915650
        public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);
        public static double maxAutoSpeed = 3.0; // meters per second
        public static double maxAutoAcceleration = 3.0; // meters per second per second
        public static double kRamseteB = 2.0;
        public static double kRamseteZeta = 0.7;
        public static double kPDriveVel = 0.0; //1.9736
        public static double maxAutoTurn = 0.55;
    }

    public static final class DriveConstants {
        /**
         * DRIVE MOTOR PORTS
         **/
        public static final int leftFrontDrivePort = 2;
        public static final int leftRearDrivePort = 1;
        public static final int rightFrontDrivePort = 3;
        public static final int rightRearDrivePort = 4;

        public static final int rightEncoderChannelA = 2;
        public static final int rightEncoderchannelB = 3;
        public static final int leftEncoderChannelA = 1;
        public static final int leftEncoderChannelB = 0;

        /**
         * MISC CONSTANTS
         **/
        public static final double trackWidth = Units.inchesToMeters(28);
        public static final double driveGearRatio = 4 / 3.0;
        public static final double wheelDiameter = 6.0; // 6 in.

        public static final double minDriveSpeed = 0.1;
        public static final double maxDriveSpeed = 0.95;
        public static final double maxAutoSpeed = 0.55;
    }

    public static final class OIConstants {
        /** XBOX CONTROLLER PORTS **/
        public static final int xboxControllerPort = 1;
        public static final int fightStickPort = 0;
    }

    public static final class MechanismConstants {
        // INDEXER
        public static final int intakeMotorPort = 6;
        public static final int indexerMotorPort = 12;
        public static final int intakeToIndexerMotorPort = 13;
        public static final double indexerSpeed = 0.25;
        public static final double intakeToIndexerSpeed = 0.2;
        public static final int intakeToIndexerResidualIndexTimeMillis = 750;
        //CLIMBER
        public static final int leftClimberMotorPort = 15;
        public static final int rightClimberMotorPort = 16;
        public static final double telescopeSpeed = 0.2;
        // TURRET
        public static final int turretMotorPort = 10;
        public static final double turretTurnSpeed = 0.08;
        // SHOOTER
        public static final int shooterMotorPortA = 7;
        public static final int shooterMotorPortB = 8;
        // SHOOTER HOOD
        public static final double minimumHoodAngle = 8;
        public static final double maximumHoodAngle = 41;
        public static final double defaultHoodAngle = 25;
        // MAX MECHANISM SPEEDS
        public static final double idleOuttakeSpeed = 0.35;
        public static final double intakeSpeed = 0.3; //0 to 1

        // Climber motor ports ...
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

        public static double maxError = 0.05;
        public static double maxControlEffort = 12;
        public static double modelDeviation = 0.075;
        public static double encoderDeviation = 0.02;
        public static double hoodToHub = 0.9207;
    }
}

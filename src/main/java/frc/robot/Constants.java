// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class AutoConstants {
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(Constants.DriveConstants.trackWidth);
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.AutoConstants.ksVolts, Constants.AutoConstants.kvVoltSecondsPerMeter, Constants.AutoConstants.kaVoltSecondsSquaredPerMeter), kDriveKinematics, 11.0);
        // ramesete params
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;


    }

    public static final class DriveConstants {
        /** DRIVE MOTOR PORTS **/
        public static final int leftFrontDrivePort = 4;
        public static final int leftRearDrivePort = 3;
        public static final int rightFrontDrivePort = 2;
        public static final int rightRearDrivePort = 1;

        /** MISC CONSTANTS **/
        public static final double trackWidth = Units.inchesToMeters(27);
        public static final int driveGearRatio = 0;
        public static final int wheelDiameter = 0; // 6 in.
        public static final double minDriveSpeed = 0.2;
        public static final double maxDriveSpeed = 0.85;
        public static final double maxDriveSpeedMetersPerSecond = 3;
        public static final double maxAccelerationMetersPerSecondSquared= 3;

    }

    public static final class OIConstants {
        /** XBOX CONTROLLER PORTS **/
        public static final int xboxControllerPort = 0;
        public static final int fightStickPort = 1;
    }

    public static final class MechanismConstants {
        public static final int intakeMotorPort = 0;  // CHANGE
        public static final int indexerMecanumMotorPort = 0;  // CHANGE
        public static final int indexerBeltMotorPort = 0;  // CHANGE
        public static final int turretMotorPort = 0;  // CHANGE
        public static final int shooterMotorPortA = 0;  // CHANGE
        public static final int shooterMotorPortB = 0;  // CHANGE
        public static final int hoodAngleMotorPort = 0;  // CHANGE
        public static final double defaultHoodAngle = 30; //CHANGE

        // Climber motor ports ...
    }

    public static final class PneumaticConstants {
        public static final int intakePneumaticPortA = 0;
        public static final int intakePneumaticPortB = 0;

        // Pneumatics constants ...
    }

    public static final class EncoderConstants {
        public static final int hoodAngleEncoderPortA = 0;
        public static final int hoodAngleEncoderPortB = 0;
    }


}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public static final class AutoConstants {

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
        public static final double minDriveSpeed = 0.1;
        public static final double maxDriveSpeed = 0.95;
        public static final double maxAutoSpeed = 0.55;
    }

    public static final class OIConstants {
        /** XBOX CONTROLLER PORTS **/
        public static final int xboxControllerPort = 0;
        public static final int fightStickPort = 1;
    }

    public static final class MechanismConstants {
        public static final int intakeMotorPort = 6;
        public static final int indexerMecanumMotorPort = 0;  // CHANGE
        public static final int indexerBeltMotorPort = 0;  // CHANGE

        public static final int turretMotorPort = 0;  // CHANGE

        public static final int shooterMotorPortA = 0;  // CHANGE
        public static final int shooterMotorPortB = 0;  // CHANGE

        public static final int hoodAngleMotorPort = 0;  // CHANGE
        public static final double defaultHoodAngle = 30; //CHANGE

        public static final double intakeSpeed = 0.4; //0 to 1

        // Climber motor ports ...
    }

    public static final class PneumaticConstants {
        public static final int shifterSolenoidPortA = 0; //3A
        public static final int shifterSolenoidPortB = 2; //3B
        public static final int intakePneumaticPortLeftA = 1; //2A
        public static final int intakePneumaticPortLeftB = 4; //2B
        public static final int pneumaticPortRightA = 7; //1A
        public static final int pneumaticPortRightB = 3; // 1B

        // Pneumatics constants ...
    }

    public static final class EncoderConstants {
        public static final int hoodAngleEncoderPortA = 0;
        public static final int hoodAngleEncoderPortB = 0;
    }


}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    public static final class AutoConstants {

    }

    public static final class DriveConstants {
        /** DRIVE MOTOR PORTS **/
        public static final int leftFrontDrivePort = 0;
        public static final int leftRearDrivePort = 1;
        public static final int rightFrontDrivePort = 3;
        public static final int rightRearDrivePort = 2;


        /** MISC CONSTANTS **/
        public static final int driveGearRatio = 0;
        public static final int wheelDiameter = 0; // 6 in.

    }

    public static final class OIConstants {
        /** XBOX CONTROLLER PORTS **/
        public static final int xboxControllerPort = 0;
        public static final int fightStickPort = 1;
    }

    public static final class MechanismConstants {
        public static final int IntakeMotorPort = 0;  // CHANGE
        public static final int shooterMotorPort = 0; // CHANGE
        public static final int IndexerMecanumMotorPort = 0;  // CHANGE

        // Climber motor ports ...
    }

    public static final class PneumaticConstants {
        public static final int IntakePneumaticPort1 = 0;
        public static final int IntakePneumaticPort2 = 0;

        // Pneumatics constants ...
    }
}

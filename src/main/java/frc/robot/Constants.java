// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import modulelib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int CONTROLLER_DRIVE_PORT = 0;
    public static final int CONTROLLER_OPERATOR_PORT = 1;
    public static final int LED_ID = 2;
    public static final int CURRENT_LIMIT = 100;

    public static final class DriveConstants {
        // All for MK4i modules
        public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;
        public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        public static final double TURNING_GEAR_RATIO = 150.0 / 7.0;
        public static final double wheelBase = Units.inchesToMeters(18.5);
        public static final double trackWidth = Units.inchesToMeters(18.5);
        // This equates to about 5.5435 m/s
        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = (1 / DRIVE_GEAR_RATIO) * (6380.0 / 60)
                * WHEEL_CIRCUMFERENCE;

        // Speeds for the robot when moving, all are in Meters/Second
        public static final double TELEOP_MAX_SPEED_METERS_PER_SECOND = 0.5; // Max is 5.5435
        public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 0.3;
        public static final double teleOpNormalAngularSpeed = 0.3;
        public static final double teleOpSlowAngularSpeed = 0.1;

        public static final double DRIVE_ENCODER_TO_METERS = (WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 2048.0));
        public static final double DRIVE_ENCODER_VELOCITY_TO_METERS_PER_SECOND = (600.0 * WHEEL_CIRCUMFERENCE)
                / (2048.0 * 60 * DRIVE_GEAR_RATIO);

        public static final double MAX_VOLTAGE = 12;
        public static final double deadband = 0.08;
        public static final int currentLimit = 40;
        public static final double slewRate = 50; // lower number for higher center of mass

        public static final class SwervePID { // TO-DO: Play around with these, they may affect auto performance
            public static final double p = 0.12;
            public static final double i = 0;
            public static final double d = 0.0015;
        }

        public static final class SwerveModules {
            public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(11, 15, 19, false);
            public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(10, 14, 18, true);
            public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(13, 17, 21, true);
            public static final SwerveModuleConfig backRight = new SwerveModuleConfig(12, 16, 20, false);
        }

        public static final class ModuleLocations {
            public static final double dist = Units.inchesToMeters(9.25);
            public static final double robotRaduius = Math.sqrt(2 * Math.pow(dist, 2));
            public static final Translation2d frontLeft = new Translation2d(dist, dist);
            public static final Translation2d frontRight = new Translation2d(dist, -dist);
            public static final Translation2d backLeft = new Translation2d(-dist, dist);
            public static final Translation2d backRight = new Translation2d(-dist, -dist);
        }
    }

    public static final class PathPlannerConstants {
        public static final class TranslationPID {
            public static final double p = 3; // 4 (Non carpet); 3 (Carpet)
            public static final double i = 0;
            public static final double d = 0;
        }

        public static final class RotationPID {
            public static final double p = 4; // 2 (Non carpet); 4 (Carpet)
            public static final double i = 0;
            public static final double d = 0;
        }
    }
}
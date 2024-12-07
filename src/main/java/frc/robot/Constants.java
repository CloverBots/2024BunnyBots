package frc.robot;

import modulelib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import limelight.LimelightConfiguration;

public final class Constants {
    public static final int CONTROLLER_DRIVE_PORT = 0;
    public static final int CONTROLLER_OPERATOR_PORT = 1;
    public static final int GYRO_ID = 5;
    public static final int INTAKE_MOTOR_ID = 6;
    public static final int PIVOT_MOTOR = 7;
    public static final int BLOWER_MOTOR_ID = 8;
    public static final int CURRENT_LIMIT = 60;

    public static final class DriveConstants {
        // All for MK4i modules
        public static final double WHEEL_RADIUS = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * Math.PI;
        public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
        public static final double KRAKEN_FREE_SPEED = 6000.0;
        public static final double wheelBase = Units.inchesToMeters(18.5);
        public static final double trackWidth = Units.inchesToMeters(18.5);
        // This equates to about 5.5435 m/s
        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = (1 / DRIVE_GEAR_RATIO)
                * (KRAKEN_FREE_SPEED / 60)
                * WHEEL_CIRCUMFERENCE; // Max is 5.2134 meter/sec

        // Max is 15.6903 rad/sec, which is ~2.5 rotations per second
        public static final double PHYSICAL_MAX_ROTATION_SPEED = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / ModuleLocations.robotRadius;

        // Set speeds for the robot when moving, in Meters/Second
        public static final double TELEOP_MAX_SPEED_METERS_PER_SECOND = 3;
        public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 2;

        // Rotation speed multiplier to the (-1, 1) input given by the joystick
        public static final double TELEOP_NORMAL_ANGULAR_SCALE_FACTOR = 0.1;
        public static final double TELEOP_SLOW_ANGULAR_SCALE_FACTOR = 0.05;

        public static final double DRIVE_ENCODER_TO_METERS = (WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 2048.0));
        public static final double DRIVE_ENCODER_VELOCITY_TO_METERS_PER_SECOND = (600.0 * WHEEL_CIRCUMFERENCE)
                / (2048.0 * 60 * DRIVE_GEAR_RATIO);

        public static final double MAX_VOLTAGE = 12;
        public static final double deadband = 0.08;
        public static final double slewRate = 10; // TO-DO this should fix the acceleration problem

        public static final class SwervePID {
            public static final double p = 0.12;
            public static final double i = 0;
            public static final double d = 0.0015;
        }

        public static final class SwerveModules {
            public static final SwerveModuleConfig frontRight = new SwerveModuleConfig(11, 15, 19, false);
            public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(10, 14, 18, false);
            public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(13, 17, 21, false);
            public static final SwerveModuleConfig backRight = new SwerveModuleConfig(12, 16, 20, false);
        }

        public static final class ModuleLocations {
            public static final double dist = Units.inchesToMeters(9.25);
            public static final double robotRadius = Math.sqrt(2 * Math.pow(dist, 2));
            public static final Translation2d frontLeft = new Translation2d(dist, dist);
            public static final Translation2d frontRight = new Translation2d(dist, -dist);
            public static final Translation2d backLeft = new Translation2d(-dist, dist);
            public static final Translation2d backRight = new Translation2d(-dist, -dist);
        }
    }

    public static final class PathPlannerConstants {
        public static final class TranslationPID {
            public static final double p = 3;
            public static final double i = 0;
            public static final double d = 0;
        }

        public static final class RotationPID {
            public static final double p = 4;
            public static final double i = 0;
            public static final double d = 0;
        }
    }

    public static final class SuperstructureConstants {
        public static final double INTAKE_SPEED = 0.3; //Increase after testing
        public static final double OUTTAKE_SPEED = -0.15; 
         public static final double OUTTAKE_TIME = 1; // seconds 
        public static final double LOWER_ENDPOINT = -19; //TO-DO set
        public static final double UPPER_ENDPOINT = 0;

        public static final double PARK_SET_POINT = -3;
        public static final double MANUAL_PARK_STOP = -1;
        public static final double SCORE_SET_POINT = -10;
        public static final double INTAKE_SET_POINT = -18;

        public static final double BLOWER_SPEED = 1;
    }

     public static final class VisonConstants {
        private static final double VISION_TARGET_HEIGHT = 7.75; // AprilTag 1-12
        private static final double CAMERA_HEIGHT = 33.25; // inches
        private static final double CAMERA_PITCH = -20; // degrees
        public final static LimelightConfiguration visionConfiguration = new LimelightConfiguration(
                VISION_TARGET_HEIGHT,
                CAMERA_HEIGHT,
                CAMERA_PITCH);
    }
}
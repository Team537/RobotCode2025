// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.swerve.DrivingMotor;
import frc.robot.util.swerve.TurningMotor;
import frc.robot.util.upper_assembly.UpperAssemblyType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants.
 * This class should not be used for any other purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * <h2>OperatorConstants</h2>
     * The {@code OperatorConstants} class is a subclass contained within the {@code Constants} class.
     * This subclass contains all of the constants relating to how the robot is manually controlled.
     * This includes things like the maximum boost mode speed, the driver controller port, driver rotational offsets, etc.
     */
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;

        // Position offsets, used to determine the orientation of the driver
        public static final Rotation2d BLUE_ALLIANCE_OFFSET = new Rotation2d(-0.5 * Math.PI);
        public static final Rotation2d RED_ALLIANCE_OFFSET = new Rotation2d(0.5 * Math.PI);
        public static final Rotation2d DEMO_ALLIANCE_OFFSET = new Rotation2d(-0.5 * Math.PI);

        // Controller Constants
        public static final double LINEAR_INPUT_CURVE_POWER = 2.5;
        public static final double ROTATION_INPUT_CURVE_POWER = 2.5;
        public static final double THROTTLE_LINEAR_MIN_SPEED = 2.0; // Meters per second
        public static final double THROTTLE_LINEAR_MAX_SPEED = DriveConstants.LINEAR_MAX_SPEED; // Meters per second
        public static final double THROTTLE_ROTATIONAL_MIN_SPEED = 7.0; // Radians per second
        public static final double THROTTLE_ROTATIONAL_MAX_SPEED = DriveConstants.ROTATIONAL_MAX_SPEED; // Radians per
                                                                                                        // second
        public static final double XBOX_CONTROLLER_JOYSTICK_DEADMAND_RADIUS = 0.01;
        public static final double XBOX_CONTROLLER_TARGET_MIN_RADIUS = 1.0; // Meters
        public static final double XBOX_CONTROLLER_TARGET_MAX_RADIUS = 5.0; // Meters
        public static final double XBOX_CONTROLLER_ROTATIONAL_TARGET_ACTIVATION_ZONE = 0.8;
        public static final double XBOX_CONTROLLER_ROTATIONAL_TARGET_DEACTIVATION_ZONE = 0.7;
    }

    /**
     * <h2>DriveConstants</h2>
     * The {@code DriveConstants} class is a subclass contained within the @code Constants} class. 
     * This subclass contains all of the constants relating to the robot's drivetrain.
     * This includes things like the turning factor, maximum drive speed, conversion factors, etc.
     */
    public final class DriveConstants {

        // Angular Offsets for Swerve Modules
        public static final Rotation2d FRONT_LEFT_MODULE_ANGULAR_OFFSET = new Rotation2d(-0.5 * Math.PI);
        public static final Rotation2d REAR_LEFT_MODULE_ANGULAR_OFFSET = new Rotation2d(Math.PI);
        public static final Rotation2d FRONT_RIGHT_MODULE_ANGULAR_OFFSET = new Rotation2d(0);
        public static final Rotation2d REAR_RIGHT_MODULE_ANGULAR_OFFSET = new Rotation2d(Math.PI / 2);

        // SPARK MAX CAN IDs for Driving Motors
        public static final int FRONT_LEFT_DRIVING_MOTOR_CAN_ID = 2;
        public static final int REAR_LEFT_DRIVING_MOTOR_CAN_ID = 1;
        public static final int FRONT_RIGHT_DRIVING_MOTOR_CAN_ID = 7;
        public static final int REAR_RIGHT_DRIVING_MOTOR_CAN_ID = 8;

        // SPARK MAX CAN IDs for Turning Motors
        public static final int FRONT_LEFT_TURNING_MOTOR_CAN_ID = 3;
        public static final int REAR_LEFT_TURNING_MOTOR_CAN_ID = 10;
        public static final int FRONT_RIGHT_TURNING_MOTOR_CAN_ID = 6;
        public static final int REAR_RIGHT_TURNING_MOTOR_CAN_ID = 9;

        // Gyroscope CAN ID
        public static final int GYROSCOPE_DEVICE_ID = 42;

        // Drivetrain Dimensions
        public static final double TRACK_WIDTH = Units.inchesToMeters(16);
        public static final double WHEEL_BASE = Units.inchesToMeters(16);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        public static final double WHEEL_RADIUS = 0.0381; // Meters
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2.0 * Math.PI; // Meters

        // Motion Constraints
        public static final double LINEAR_MAX_SPEED = 4.8; // m/s
        public static final double ROTATIONAL_MAX_SPEED = 16.7; // rad/s
        public static final double LINEAR_MAX_ACCELERATION = 11.4; // m/s²
        public static final double ROTATIONAL_MAX_ACCELERATION = 42.0; // rad/s²
        public static final double MAX_DELTA_TIME_RATE_LIMIT = 0.1; // Prevents excessive acceleration due to lag

        // Drive PID Controller Coefficients
        public static final double LINEAR_KP = 1.0;
        public static final double LINEAR_KI = 0.0;
        public static final double LINEAR_KD = 0.0;

        public static final double ROTATIONAL_KP = 0.5;
        public static final double ROTATIONAL_KI = 0.0;
        public static final double ROTATIONAL_KD = 0.2;

        // Default motor values
        public static final DrivingMotor DEFAULT_DRIVING_MOTOR = DrivingMotor.NEO;
        public static final TurningMotor DEFAULT_TURNING_MOTOR = TurningMotor.NEO_550;

        /*
         * ---------------------------------- DRIVING CONSTANTS ----------------------------------
         */
        public static final class NeoDriving {
            public static final double FREE_SPEED = 594.389; // Rad/s
            public static final double MOTOR_REDUCTION = 4.714;
            public static final double DRIVE_WHEEL_FREE_SPEED = (FREE_SPEED * WHEEL_RADIUS) / MOTOR_REDUCTION;

            public static final double ENCODER_POSITION_FACTOR = WHEEL_CIRCUMFERENCE / MOTOR_REDUCTION;
            public static final double ENCODER_VELOCITY_FACTOR = ENCODER_POSITION_FACTOR / 60.0;

            public static final double KP = 0.04;
            public static final double KI = 0;
            public static final double KD = 0;
            public static final double FF = 1 / DRIVE_WHEEL_FREE_SPEED;
            public static final double PID_MIN_OUTPUT = -1;
            public static final double PID_MAX_OUTPUT = 1;

            public static final IdleMode IDLE_MODE = IdleMode.kBrake;
            public static final int CURRENT_LIMIT = 50; // Amps
        }

        public static final class KrakenX60Driving {
            public static final double FREE_SPEED = 628.319; // Rad/s
            public static final double MOTOR_REDUCTION = 4.714;
            public static final double DRIVE_WHEEL_FREE_SPEED = (FREE_SPEED * WHEEL_RADIUS) / MOTOR_REDUCTION;

            public static final double SENSOR_TO_MECHANISM_RATIO = MOTOR_REDUCTION / WHEEL_CIRCUMFERENCE;

            public static final double KP = 1.0;
            public static final double KI = 1.0;
            public static final double KD = 0.01;
            public static final double KV = 2.36;
            public static final double KA = 0.18;

            public static final double PID_MIN_OUTPUT = -1;
            public static final double PID_MAX_OUTPUT = 1;

            public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
            public static final boolean CURRENT_LIMIT_ENABLED = true;
            public static final int CURRENT_LIMIT = 75; // Amps
            public static final int CURRENT_LOWER_LIMIT = 25;
            public static final double CURRENT_LOWER_TIME = 0.5;
        }

        public static final class KrakenX60FOCDriving {
            public static final double FREE_SPEED = 607.375; // Example value
            public static final double MOTOR_REDUCTION = 4.714; // Example value
            public static final double DRIVE_WHEEL_FREE_SPEED = (FREE_SPEED * WHEEL_RADIUS) / MOTOR_REDUCTION;

            public static final double SENSOR_TO_MECHANISM_RATIO = MOTOR_REDUCTION / WHEEL_CIRCUMFERENCE;

            public static final double KP = 1.0;
            public static final double KI = 1.0;
            public static final double KD = 0.01;
            public static final double KV = 2.44;
            public static final double KA = 0.13;

            public static final double PID_MIN_OUTPUT = -1;
            public static final double PID_MAX_OUTPUT = 1;

            public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
            public static final boolean CURRENT_LIMIT_ENABLED = true;
            public static final int CURRENT_LIMIT = 75; // Amps
            public static final int CURRENT_LOWER_LIMIT = 25;
            public static final double CURRENT_LOWER_TIME = 0.5;
        }

        /*
         * ---------------------------------- TURNING CONSTANTS ----------------------------------
         */
        public static final class Neo550Turning {
            public static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // Radians
            public static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // Rad/s
            public static final double POSITION_PID_MIN_INPUT = 0.0;
            public static final double POSITION_PID_MAX_INPUT = ENCODER_POSITION_FACTOR;

            public static final boolean ENCODER_INVERTED = true;

            public static final double KP = 1.0;
            public static final double KI = 0.0;
            public static final double KD = 0.0;
            public static final double FF = 0.0;
            public static final double PID_MIN_OUTPUT = -1.0;
            public static final double PID_MAX_OUTPUT = 1.0;

            public static final IdleMode IDLE_MODE = IdleMode.kBrake;
            public static final int CURRENT_LIMIT = 20; // Amps
        }
    }

    /**
     * <h2>UpperAssemblyConstants</h2>
     * The {@code UpperAssemblyConstants} class is a subclass contained within the {@code Constants} class.
     * This subclass contains all of the constants relating to both team's mechanisms.
     * This contains values like the default upper assembly.
     */
    public static class UpperAssemblyConstants {
        public static final UpperAssemblyType DEFAULT_UPPER_ASSEMBLY = UpperAssemblyType.SQUID;
    }

    /**
     * <h2>NarwhalConstants</h2>
     * The {@code NarwhalConstants} class is a subclass contained within the {@code Constants} class.
     * This subclass contains all of the constants relating to the Narwhal's mechanisms.
     * This contains values like motor IDs, PID coefficients, etc.
     */
    public static class NarwhalConstants {
        public static class NarwhalIntakeOuttakeConstants {
            public static final int INTAKE_OUTTAKE_MOTOR_CAN_ID = 12;
            public static final int INTAKE_OUTTAKE_MOTOR_CURRENT_LIMIT = 40;
            // Settings for % of max power to use on intake and outtake
            public static final double INTAKE_MOTOR_PERCENT = 0.3; // between -1.0 and 1.0
            public static final double OUTTAKE_MOTOR_PERCENT = -0.5; // between -1.0 and 1.0

            public static final double POSITION_PID_P = 0.7;
            public static final double POSITION_PID_I = 0;
            public static final double POSITION_PID_D = 0.2;

            public static final double PID_MAX_OUTPUT = 0.5;
            public static final double PID_MIN_OUTPUT = -0.5;
        }

        public static class NarwhalWristConstants {
            public static final int WRIST_MOTOR_CAN_ID = 8;
            public static final int WRIST_MOTOR_CURRENT_LIMIT = 20;
            public static final double WRIST_OFFSET = 0.0; // offset will be calculated as if unflipped, and no conversion factor.

            public static final double ROTATIONS_TO_RADIANS = Math.PI * 2; // Wrist target angles (radians) are multiplied by this to get the motor target position

            public static final double POSITION_PID_P = 0.5; // TODO: UPDATE THESE PID VALUES
            public static final double POSITION_PID_I = 0; // TODO: UPDATE THESE PID VALUES
            public static final double POSITION_PID_D = 0.2; // TODO: UPDATE THESE PID VALUES

            public static final double PID_OUTPUT_RANGE_MAX = 0.5; // TODO: UPDATE THESE OUTPUT RANGE VALUES
            public static final double PID_OUTPUT_RANGE_MIN = 0.5; // TODO: UPDATE THESE OUTPUT RANGE VALUES

            public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromRadians(-Math.PI / 4); // -pi/4 TODO: update these placeholder values
            public static final Rotation2d OUTTAKE_ANGLE = Rotation2d.fromRadians(2 * Math.PI / 3); // 2pi/3 TODO: update these placeholder values
            public static final Rotation2d ALGAE_ANGLE = Rotation2d.fromRadians(Math.PI / 2); // pi/2 TODO: update these placeholder values
        }

        public static class NarwhalClimberConstants {
            public static final int CLIMBER_CAN_ID = 15;

            public static final boolean IS_CLIMBER_INVERTED = false;

            public static final double GEAR_REDUCTION = 125.0;
            public static final double PULLY_REDUCTION = 10.0;
            public static final double CLIMBER_ANGLE_TO_MOTOR_ANGLE = GEAR_REDUCTION * PULLY_REDUCTION;

            public static final double PID_P = 5;
            public static final double PID_I = 0;
            public static final double PID_D = 0.1;
            public static final double PID_F = 1.9;

            public static final double CLIMBER_PID_MIN_OUTPUT = -0.3;
            public static final double CLIMBER_PID_MAX_OUTPUT = 0.3;

            public static final Rotation2d DEPLOYED_ANGLE = Rotation2d.fromDegrees(30);
            public static final Rotation2d CLIMB_ANGLE = Rotation2d.fromDegrees(-5);
        }
    }

    /**
     * <h2>SquidConstants</h2>
     * The {@code SquidConstants} class is a subclass contained within the {@code Constants} class.
     * This subclass contains all of the constants relating to the Squids's mechanisms.
     * This contains values like motor IDs, PID coefficients, etc.
     */
    public static class SquidConstants {
        public static class SquidManipulatorConstants {

            // TODO: Put actual IDs in
            public static final int TOP_MOTOR_CAN_ID = 11;
            public static final int BOTTOM_MOTOR_CAN_ID = 12;

            public static final int CORAL_SENSOR_ID = 0;

            public static final double KP = 0.005;
            public static final double KI = 0.0001;
            public static final double KD = 0;
            public static final double FF = 0.01;

            public static final double PID_MIN_OUTPUT = -1.0;
            public static final double PID_MAX_OUTPUT = 1.0;

            public static final double WHEEL_RADIUS = 0.038;
            public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // Meters
            public static final double MOTOR_REDUCTION = 20.0;
            public static final double ENCODER_FACTOR = WHEEL_CIRCUMFERENCE / MOTOR_REDUCTION;
            public static final double MANIPULATOR_MOTOR_FREE_SPEED = 1151.917; // Radians / second

            public static final double MANIPULATOR_MAX_SPEED = (MANIPULATOR_MOTOR_FREE_SPEED * WHEEL_RADIUS) / MOTOR_REDUCTION;

            public static final boolean TOP_MOTOR_INVERTED = false;
            public static final boolean BOTTOM_MOTOR_INVERTED = false;

            public static final int MOTOR_CURRENT_LIMIT = 20; // Amps
            public static final IdleMode IDLE_MODE = IdleMode.kBrake;

            // TODO: Add the actual values
            public static final double L_ONE_BOTTOM_ROLLER_RATIO = 1.0;
            public static final double L_TWO_BOTTOM_ROLLER_RATIO = 0.95;
            public static final double L_THREE_BOTTOM_ROLLER_RATIO = 0.95;
            public static final double L_FOUR_BOTTOM_ROLLER_RATIO = 0.6;

            public static final double OUTTAKE_MAX_DISTANCE = 0.5;
        }

        public static class SquidClimberConstants {

            // IDs
            public static final int CLIMB_MOTOR_CAN_ID = 15;

            // Behavior Constants
            public static final boolean MOTOR_INVERTED = false;
            public static final int MOTOR_CURRENT_LIMIT = 50;
            public static final IdleMode IDLE_MODE = IdleMode.kBrake;

            // Gearing Constants
            public static final double SPOOL_RADIUS = 0.009526; // Meters
            public static final double SPOOL_CIRCUMFERENCE = 2.0 * Math.PI * SPOOL_RADIUS; // Meters
            public static final double MOTOR_REDUCTION = 125.0;
            public static final double ENCODER_FACTOR = SPOOL_CIRCUMFERENCE / MOTOR_REDUCTION;

            // PID Values
            public static final int KP = 1;
            public static final int KI = 0;
            public static final int KD = 0;

            public static final double PID_MIN_OUTPUT = -1.0;
            public static final double PID_MAX_OUTPUT = 1.0;

            /**
             * The spool length of the down position, in meters. The further the distance,
             * the more spool is pulled back.
             */
            public static final double DOWN_POSITION = 0.0; // Meters

            /**
             * The spool length of the climbed position, in meters. The further the
             * distance, the more spool is pulled back.
             */
            public static final double CLIMBED_POSITION = 0.203; // Meters

        }
    }

    /**
     * <h2>VisionConstants</h2>
     * The {@code VisionConstants} class is a subclass contained within the {@code Constants} class.
     * This subclass contains all of the constants relating to the robot's vision solution.
     * This contains values such as the PI's IP, camera names, AprilTag field Layout, etc.
     */
    public static class VisionConstants {

        // Camera Settings
        public static final String FRONT_CAMERA_NAME = "Front_Camera";
        public static final String SLIDE_CAMERA_NAME = "Side_Camera";

        public static final Transform3d FRONT_CAMERA_OFFSET = new Transform3d(); // TODO: Fill in actual values.
        public static final Transform3d SLIDE_CAMERA_OFFSET = new Transform3d(); // TODO: Fill in actual values.

        // Pipeline settings
        public static final int APRIL_TAG_PIPELINE = 0;
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025Reefscape);

        // Odometry Detection Strategy
        public static final PhotonPoseEstimator.PoseStrategy POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        public static final PhotonPoseEstimator.PoseStrategy FALLBACK_STRATEGY = PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
    }

    /**
     * <h2>OceanViewConstants</h2>
     * The {@code OceanViewConstants} class is a subclass contained within the {@code Constants} class.
     * This subclass contains all of the constants relating to OceanView, whether it be ports or IPs for coprocessors.
     */
    public static class OceanViewConstants {
        public static final String PI_IP = "10.5.37.57";
        public static final int DASHBOARD_PORT_NUMBER = 5000;
        public static final int UDP_PORT_NUMBER = 5400;
        public static final int TCP_PORT_NUMBER = 5300;
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import org.photonvision.PhotonPoseEstimator;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
     * The {@code DriveConstants} class is a subclass contained within the @code Constants} class. This subclass
     * contains all of the constants relating to the robot's drivetrain. 
     * This includes things like the turning factor, maximum drive speed, conversion factors, etc.
     */
    public static class DriveConstants {

        // Controller Constants
        public static final double LINEAR_INPUT_CURVE_POWER = 2.5;
        public static final double ROTATION_INPUT_CURVE_POWER = 2.5;
        public static final double TURNING_FACTOR = 2.0 * Math.PI;

        // Angular offsets of the modules relative to the chassis in radians
        public static final Rotation2d FRONT_LEFT_MODULE_ANGULAR_OFFSET = new Rotation2d(-0.5 * Math.PI);
        public static final Rotation2d REAR_LEFT_MODULE_ANGULAR_OFFSET = new Rotation2d(Math.PI);
        public static final Rotation2d FRONT_RIGHT_MODULE_ANGULAR_OFFSET = new Rotation2d(0);
        public static final Rotation2d REAR_RIGHT_MODULE_ANGULAR_OFFSET = new Rotation2d(Math.PI / 2);

        // SPARK MAX CAN IDs

        // Drive Motor IDs
        public static final int FRONT_LEFT_DRIVING_MOTOR_CAN_ID = 2;
        public static final int REAR_LEFT_DRIVING_MOTOR_CAN_ID = 1;
        public static final int FRONT_RIGHT_DRIVING_MOTOR_CAN_ID = 7;
        public static final int REAR_RIGHT_DRIVING_MOTOR_CAN_ID = 8;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED = 94.6; // Revolutions per second
        public static final double WHEEL_DIAMETER = 0.06677; // Meters
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // Meters
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = 4.714;
        public static final double DRIVE_WHEEL_FREE_SPEED = (DRIVING_MOTOR_FREE_SPEED * WHEEL_CIRCUMFERENCE)
                / DRIVING_MOTOR_REDUCTION; // Meters / Second

        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI)
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER * Math.PI)
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final boolean DRIVING_ENCODER_INVERTED = false;

        // PIDs for the Driving Motor
        public static final double DRIVING_KP = 0.04;
        public static final double DRIVING_KI = 0;
        public static final double DRIVING_KD = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED;
        public static final double DRIVING_PID_MIN_OUTPUT = -1;
        public static final double DRIVING_PID_MAX_OUTPUT = 1;

        public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps

        // Turning Motor IDs
        public static final int FRONT_LEFT_TURNING_MOTOR_CAN_ID = 3;
        public static final int REAR_LEFT_TURNING_MOTOR_CAN_ID = 10;
        public static final int FRONT_RIGHT_TURNING_MOTOR_CAN_ID = 6;
        public static final int REAR_RIGHT_TURNING_MOTOR_CAN_ID = 9;

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final boolean TURNING_ENCODER_INVERTED = true;

        // PIDs for the turning Motor
        public static final double TURNING_KP = 1;
        public static final double TURNING_KI = 0;
        public static final double TURNING_KD = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_PID_MIN_OUTPUT = -1.0;
        public static final double TURNING_PID_MAX_OUTPUT = 1.0;

        public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // Amps

        // Gyroscope CAN IDs
        public static final int GYROSCOPE_DEVICE_ID = 42;

        // Drive PID Controller Coefficients
        public static final double LINEAR_KP = 1.0;
        public static final double LINEAR_KI = 0.0;
        public static final double LINEAR_KD = 0.0;

        public static final double ROTATIONAL_KP = 0.5;
        public static final double ROTATIONAL_KI = 0.0;
        public static final double ROTATIONAL_KD = 0.2;

        // Maxmimum speeds and accelerations for driving
        public static final double LINEAR_MAX_SPEED = 4.8; // Meters per second
        public static final double ROTATIONAL_MAX_SPEED = 16.7; // Radians per second
        public static final double LINEAR_MAX_ACCELERATION = 11.4; // Meters per second squared
        public static final double ROTATIONAL_MAX_ACCELERATION = 42.0; // Radians per second squared

        // Maximum delta time for driving, prevents too fast accelerations when lag
        // occurs.
        public static final double MAX_DELTA_TIME_RATE_LIMIT = 0.1; // 1.0; // seconds

        // The drivetrain size and kinematics

        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(16);

        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(16);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));
      }

    /**
     * <h2> NarwhalConstants </h2>
     * The {@code NarwhalConstants} class is a subclass contained within the {@code Constants} class.
     * This subclass contains all of the constants relating to  the Narwhal's mechanisms. 
     * This contains values like motor IDs, PID coefficients, etc.
     */
    public static class NarwhalConstants {
        public static class NarwhalIntakeOuttakeConstants {
            public static final int INTAKE_OUTTAKE_MOTOR_CAN_ID = 12; // TODO: Replace this placeholder with the actual
                                                                      // id
            public static final int INTAKE_OUTTAKE_MOTOR_CURRENT_LIMIT = 20;
            public static final double INTAKE_MOTOR_PERCENTAGE = 0.35; // between -1.0 and 1.0
            public static final double OUTTAKE_MOTOR_PERCENTAGE = -0.35; // between -1.0 and 1.0

            public static final double POSITION_PID_P = 0.7;
            public static final double POSITION_PID_I = 0;
            public static final double POSITION_PID_D = 0.2;
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
          public static final Rotation2d ALGAE_ANGLE =  Rotation2d.fromRadians(Math.PI / 2); // pi/2 TODO: update these placeholder values
        }
    }

    public static class SquidConstants {
        
        public static class SquidManipulatorConstants {

            //TODO: Put actual IDs in
            public static final int TOP_MOTOR_CAN_ID = 11;
            public static final int BOTTOM_MOTOR_CAN_ID = 12;

            public static final int CORAL_SENSOR_ID = 0;

            public static final int KP = 1;
            public static final int KI = 0;
            public static final int KD = 0;

            public static final double PID_MIN_OUTPUT = -1.0;
            public static final double PID_MAX_OUTPUT = 1.0;

            public static final double WHEEL_RADIUS = 0.038;
            public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; //Meters
            public static final double MOTOR_REDUCTION = 5.0;
            public static final double ENCODER_FACTOR = WHEEL_CIRCUMFERENCE / MOTOR_REDUCTION;
            public static final double MANIPULATOR_MOTOR_FREE_SPEED = 1151.917; //Radians / second

            public static final double MANIPULATOR_MAX_SPEED = (MANIPULATOR_MOTOR_FREE_SPEED * WHEEL_RADIUS) / MOTOR_REDUCTION;

            public static final boolean TOP_MOTOR_INVERTED = true;
            public static final boolean BOTTOM_MOTOR_INVERTED = false;

            public static final int MOTOR_CURRENT_LIMIT = 20; // Amps
            public static final IdleMode IDLE_MODE = IdleMode.kBrake;

            //TODO: Add the actual values
            public static final double L_ONE_BOTTOM_ROLLER_RATIO = 1.0;
            public static final double L_TWO_BOTTOM_ROLLER_RATIO = 0.95;
            public static final double L_THREE_BOTTOM_ROLLER_RATIO = 0.95;
            public static final double L_FOUR_BOTTOM_ROLLER_RATIO = 0.6;
            
        }

        public static class SquidClimberConstants {

            //IDs
            public static final int CLIMB_MOTOR_CAN_ID = 15;

            //Behavior Constants
            public static final boolean MOTOR_INVERTED =  false;
            public static final int MOTOR_CURRENT_LIMIT = 50;
            public static final IdleMode  IDLE_MODE = IdleMode.kBrake;

            //Gearing Constants
            public static final double SPOOL_RADIUS = 0.009526; // Meters
            public static final double SPOOL_CIRCUMFERENCE = 2.0 * Math.PI * SPOOL_RADIUS; //Meters
            public static final double MOTOR_REDUCTION = 125.0;
            public static final double ENCODER_FACTOR = SPOOL_CIRCUMFERENCE / MOTOR_REDUCTION;

            //PID Values
            public static final int KP = 1;
            public static final int KI = 0;
            public static final int KD = 0;

            public static final double PID_MIN_OUTPUT = -1.0;
            public static final double PID_MAX_OUTPUT = 1.0;

            /**
             * The spool length of the down position, in meters. The further the distance, the more spool is pulled back.
             */
            public static final double DOWN_POSITION = 0.0; //Meters

            /**
             * The spool length of the climbed position, in meters. The further the distance, the more spool is pulled back.
             */
            public static final double CLIMBED_POSITION = 0.203; //Meters

        }

    }

    /**
     * <h2>VisionConstants</h2>
     * The {@code VisionConstants} class is a subclass contained within the {@code Constants} class. 
     * This subclass contains all of the constants relating to the robot's vision solution. 
     * This contains values such as the PI's IP, camera names, AprilTag field layout, etc.
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
}
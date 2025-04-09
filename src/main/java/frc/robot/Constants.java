// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.swerve.DrivingMotorType;
import frc.robot.util.swerve.TurningMotorType;
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

    public static class Defaults {

        // Default motor values
        public static final DrivingMotorType DEFAULT_DRIVING_MOTOR = DrivingMotorType.NEO;
        public static final TurningMotorType DEFAULT_TURNING_MOTOR = TurningMotorType.NEO_550;
        public static final UpperAssemblyType DEFAULT_UPPER_ASSEMBLY = UpperAssemblyType.NONE;

    }

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
        public static final Rotation2d DEMO_ALLIANCE_OFFSET = new Rotation2d(0.0);

        // Controller Constants
        public static final double LINEAR_INPUT_CURVE_POWER = 2.5;
        public static final double ROTATION_INPUT_CURVE_POWER = 2.5;
        public static final double NORMAL_LINEAR_MAX_SPEED = 2.0; // Meters/per second
        public static final double THROTTLE_LINEAR_MAX_SPEED = DriveConstants.LINEAR_MAX_SPEED; // Meters per second
        public static final double SLOW_LINEAR_MAX_SPEED = 0.5; // Meters/per second
        public static final double NORMAL_ROTATIONAL_MAX_SPEED = 7.0; // Radians per second
        public static final double THROTTLE_ROTATIONAL_MAX_SPEED = DriveConstants.ROTATIONAL_MAX_SPEED; // Radians per second
        public static final double SLOW_ROTATIONAL_MAX_SPEED = 2.0; // Meters/per second
        public static final double XBOX_CONTROLLER_JOYSTICK_DEADBAND_RADIUS = 0.01;
        public static final double XBOX_CONTROLLER_TARGET_RADIUS = 2.0; // Meters
        public static final double XBOX_CONTROLLER_TARGET_THROTTLE_RADIUS = 0.5; // Meters
        public static final double XBOX_CONTROLLER_ROTATIONAL_TARGET_ACTIVATION_ZONE = 0.99;
        public static final double XBOX_CONTROLLER_ROTATIONAL_TARGET_DEACTIVATION_ZONE = 0.7;
    }

    /**
     * <h2>DriveConstants</h2>
     * The {@code DriveConstants} class is a subclass contained within the @code Constants} class. 
     * This subclass contains all of the constants relating to the robot's drivetrain.
     * This includes things like the turning factor, maximum drive speed, conversion factors, etc.
     */
    public final class DriveConstants {

        // Robot Features
        public static final double DRIVETRAIN_MASS = 39.5; //Kg
        public static final double DRIVETRAIN_MOI = 2.285; //Kg meters
        public static final double GRAVITY_ACCELERATION = 9.81; // Meters / sec^2

        public static final double TRANSLATION_THRESHOLD = 0.05; // Meters
        public static final double ROTATION_THRESHOLD = 0.157; // Radians

        public static final double NARWHAL_CAN_RAISE_LIFT_DISTANCE = 1.0; // Meters
        public static final Transform2d NARWHAL_RAKE_ALAGE_TRANSFORM = new Transform2d(0.5,0.0,new Rotation2d());

        public static final double[] DRIVE_STANDARD_DEVIATION_COEFFICIENTS = {
            0.006611986432, 0.3500199104, 0

        };

        public static final double AUTO_DRIVING_TRANSLATIONAL_SPEED_SAFETY_FACTOR = 0.25;
        public static final double AUTO_DRIVING_TRANSLATIONAL_ACCELERATION_SAFETY_FACTOR = 0.5;
        public static final double AUTO_DRIVING_ROTATIONAL_SPEED_SAFETY_FACTOR = 0.25;
        public static final double AUTO_DRIVING_ROTATIONAL_ACCELERATION_FACTOR = 0.5;
        
        public static final Matrix<N3, N1> DRIVE_STANDARD_DEVIATION = new Matrix<>(N3.instance, N1.instance, DRIVE_STANDARD_DEVIATION_COEFFICIENTS);
    
        public static final List<Integer> AVAILABLE_SENTINEL_TAGS = List.of(
            6,
            7,
            8,
            9,
            10,
            11,
            17,
            18,
            19,
            20,
            21,
            22
        );
        public static final double SENTINEL_DISTANCE_WEIGHT = 1.0;
        public static final double SENTINEL_ORIENTATION_WEIGHT = 0.1;

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

        // Define module locations relative to the robot's center
        public static final Translation2d FRONT_LEFT_POSITION = 
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
        public static final Translation2d FRONT_RIGHT_POSITION = 
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
        public static final Translation2d BACK_LEFT_POSITION = 
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
        public static final Translation2d BACK_RIGHT_POSITION = 
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);

        // Alternatively, you can define an array containing all four positions:
        public static final List<Translation2d> MODULE_POSITIONS = List.of(
            FRONT_LEFT_POSITION,
            FRONT_RIGHT_POSITION,
            BACK_LEFT_POSITION,
            BACK_RIGHT_POSITION
        );

        public static final double WHEEL_RADIUS = 0.0381; // Meters
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2.0 * Math.PI; // Meters
        public static final double WHEEL_COEFFICIENT_FRICTION = 1.0;
    
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

        public static final double ROTATIONAL_KP = 2.0;
        public static final double ROTATIONAL_KI = 0.0;
        public static final double ROTATIONAL_KD = 0.1;

        // Update times
        public static final int SENSOR_UPDATE_TIME_HZ = 250;

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
            public static final double KS = 0;

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

            public static final double FREE_SPEED = 1151.917; // Rad/s

            public static final double MOTOR_REDUCTION = 46.42;
            public static final double MAX_TURNING_SPEED = FREE_SPEED / MOTOR_REDUCTION;

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

    public static class UpperAssemblyConstants {

        /**
         * The amount of time, in seconds, that the stub upper assembly will take to complete a simulated "task"
         */
        public static final double STUB_SIMULATED_TASK_TIME = 2.0;

    }

    /**
     * <h2>NarwhalConstants</h2>
     * The {@code NarwhalConstants} class is a subclass contained within the {@code Constants} class.
     * This subclass contains all of the constants relating to the Narwhal's mechanisms.
     * This contains values like motor IDs, PID coefficients, etc.
     */
    public static class NarwhalConstants {

        public static final double UPPER_ASSEMBLY_MASS = 17.2; //Kg
        public static final double UPPER_ASSEMBLY_MOI = 0.995; //Kg m^2

        public static final Transform2d INTAKING_RELATIVE_TRANSFORM = new Transform2d(new Translation2d(0.0,-0.019),new Rotation2d(Math.PI));
        public static Transform2d SCORING_RELATIVE_TRANSFORM = new Transform2d(new Translation2d(0.1524,0.019),new Rotation2d(0));
        public static final Transform2d ALGAE_REMOVAL_RELATIVE_TRANSFORM = new Transform2d(new Translation2d(0.0,-0.019),new Rotation2d(0.0));
        public static final Transform2d CLIMB_RELATIVE_TRANSFORM = new Transform2d(new Translation2d(0.0,0.0),new Rotation2d(Math.PI));
 

        public static class NarwhalIntakeOuttakeConstants {
            // Sensor IDs
            public static final int CORAL_LIMIT_SWITCH_PORT = 0; // Should be 0-9

            // Motor IDs
            public static final int INTAKE_OUTTAKE_MOTOR_CAN_ID = 12;
            public static final int INTAKE_OUTTAKE_MOTOR_CURRENT_LIMIT = 40;

            // Settings for % of max power to use on intake and outtake
            public static final double INTAKE_MOTOR_PERCENT = -0.3; // between -1.0 and 1.0
            public static final double OUTTAKE_MOTOR_PERCENT = 0.5; // between -1.0 and 1.0

            // PID configurations
            public static final double POSITION_PID_P = 0.7;
            public static final double POSITION_PID_I = 0;
            public static final double POSITION_PID_D = 0.2;

            public static final double PID_MAX_OUTPUT = 0.5;
            public static final double PID_MIN_OUTPUT = -0.5;

            // Coral intake detection delay - used in the CoralIntakeCommand for auto to prevent the command from ending too early
            public static final double CORAL_INTAKE_DETECTION_DELAY = 0.1; // seconds
            public static final double CORAL_OUTTAKE_DETECTION_DELAY = 0.35; // seconds
        }

        public static class NarwhalWristConstants {
            public static final int WRIST_MOTOR_CAN_ID = 14;
            public static final int WRIST_MOTOR_CURRENT_LIMIT = 40;

            // Calculating encoder conversion factor
            public static final double GEAR_REDUCTION = 20;
            public static final double ENCODER_FACTOR = Math.PI * 2 / GEAR_REDUCTION; // Wrist target angles (radians) are multiplied by this to get the motor target position           
            
            // PID configurations
            public static final double POSITION_PID_P = 0.3;
            public static final double POSITION_PID_I = 0.0;
            public static final double POSITION_PID_D = 0.35;
            public static final double POSITION_PID_F = 0.0;
            public static final double PID_OUTPUT_RANGE_MAX = 0.45;
            public static final double PID_OUTPUT_RANGE_MIN = -0.4;        

            // Set position for wrist angles (Angle is relative to the world, with 0 being the down position and rotating away from 0 being positive)
            public static Rotation2d INTAKE_ANGLE = Rotation2d.fromDegrees(50); // there's a lot of slop so this is going to be ~10 degreses over the actual position
            public static Rotation2d L1_OUTTAKE_ANGLE = Rotation2d.fromRadians(1.12 * Math.PI);   
            public static Rotation2d L2_OUTTAKE_ANGLE = Rotation2d.fromRadians(1.12 * Math.PI);
            public static Rotation2d L3_OUTTAKE_ANGLE = Rotation2d.fromRadians(1.12 * Math.PI);
            public static Rotation2d L4_OUTTAKE_ANGLE = Rotation2d.fromRadians(1.12 * Math.PI);
            public static final Rotation2d CLIMB_ANGLE = Rotation2d.fromRadians(1.12 * Math.PI); // This is the angle the wrist should be at when climbing
            public static final Rotation2d ALGAE_ANGLE =  Rotation2d.fromRadians(1.12 * Math.PI);
            public static final Rotation2d TRANSIT_ANGLE = Rotation2d.fromRadians(0.5 * Math.PI);
            
            /** The angle tolerance for the wrxist to be considered at a specific state. */
            public static final Rotation2d WRIST_ANGLE_TOLERANCE = Rotation2d.fromRadians(0.2 * Math.PI);
        }

        public static class NarwhalClimberConstants {
            public static final int CLIMBER_CAN_ID = 13;

            public static final boolean IS_CLIMBER_INVERTED = false;

            public static final double GEAR_REDUCTION = 125.0;
            public static final double CLIMBER_ANGLE_TO_MOTOR_ANGLE = GEAR_REDUCTION; // technically not a 1 to 1 conversion because of how the climber arm and winch are linked

            public static final double PID_P = 8.5;
            public static final double PID_I = 0;
            public static final double PID_D = 0.1;
            public static final double PID_F = 1.9;

            public static final double CLIMBER_PID_MIN_OUTPUT = -1.0;
            public static final double CLIMBER_PID_MAX_OUTPUT = 1.0;
            
            public static Rotation2d DEPLOYED_WINCH_ROTATIONS = Rotation2d.fromDegrees(390);
            public static Rotation2d CLIMB_WINCH_ROTATIONS = Rotation2d.fromDegrees(-350);

            /** The angle tolerance for the climber to be considered at a specific state. */
            public static final Rotation2d CLIMBER_ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);
        }

        public static class NarwhalElevatorConstants {
            public static final int ELEVATOR_LEAD_MOTOR_CAN_ID = 15;
            public static final int ELEVATOR_LEAD_MOTOR_CURRENT_LIMIT = 40;

            public static final int ELEVATOR_FOLLOWER_CAN_ID = 16;
            public static final int ELEVATOR_FOLLOWER_MOTOR_CURRENT_LIMIT = ELEVATOR_LEAD_MOTOR_CURRENT_LIMIT; // same motor so probably should use same current limit.
            
            // Calculating the ratio of rotations to distance
            /** Meters */
            private static final double ELEVATOR_GEAR_RADIUS = 0.065; // Meters
            private static final double ELEVATOR_GEAR_CIRCUMFERENCE = ELEVATOR_GEAR_RADIUS * 2.0 * Math.PI; // Meters
            private static final double MOTOR_GEAR_REDUCTION = 12.0;
            public static final double ENCODER_FACTOR = ELEVATOR_GEAR_CIRCUMFERENCE / MOTOR_GEAR_REDUCTION; // for every one rotation of the encoder, how many meters does the lift move

            // PID
            public static final double ELEVATOR_KP = 2.5;
            public static final double ELEVATOR_KI = 0;
            public static final double ELEVATOR_KD = 0.4;
            public static final double ELEVATOR_MIN_OUTPUT = -1.0;
            public static final double ELEVATOR_MAX_OUTPUT = 1.0;

            // Set positions for the length the elevator needs to extend to to score.
            public static final double MIN_HEIGHT_METERS = 0.0; // probably should leave at 0.0.
            public static final double MAX_HEIGHT_METERS = 2.1336; // stops the robot from ending itself
            public static double L1_ELEVATOR_HEIGHT = 0.05; // Meters
            public static double L2_ELEVATOR_HEIGHT = 0.05; // Meters
            public static double L3_ELEVATOR_HEIGHT = 0.6; // Meters
            public static double L4_ELEVATOR_HEIGHT = 1.6; // Meters
            public static double INTAKE_ELEVATOR_HEIGHT_METERS = 0.08; // Meters
            public static final boolean MOTOR_INVERTED = true;
            public static final double ELEVATOR_POSITION_TOLERANCE = 0.05; // Meters
            public static final double ALGAE_DESCORE_HEIGHT_METERS_TOP_ROW = -0.4; // Meters (negative to go down)
            public static final double ALGAE_DESCORE_HEIGHT_METERS_BOTTOM_ROW = 0.7; // Meters
            public static final double ALGAE_DESCORE_HEIGHT_METERS_DOWN_OFFSET = 1.7; // Meters
        }
    }

    /**
     * <h2>SquidConstants</h2>
     * The {@code SquidConstants} class is a subclass contained within the {@code Constants} class.
     * This subclass contains all of the constants relating to the Squids's mechanisms.
     * This contains values like motor IDs, PID coefficients, etc.
     */
    public static class SquidConstants {

        public static final double UPPER_ASSEMBLY_MASS = 0.0;
        public static final double UPPER_ASSEMBLY_MOI = 0.0; //Kg m^2
        public static final Transform2d INTAKING_RELATIVE_TRANSFORM = new Transform2d(new Translation2d(0.5,0.0),new Rotation2d(Math.PI));
        public static final Transform2d SCORING_RELATIVE_TRANSFORM = new Transform2d(new Translation2d(0.5,0.0),new Rotation2d(0));
        public static final Transform2d CLIMB_RELATIVE_TRANSFORM = new Transform2d(new Translation2d(0.5,0.0),new Rotation2d(Math.PI));

        public static class SquidManipulatorConstants {

            // TODO: Put actual IDs in
            public static final int TOP_MOTOR_CAN_ID = 11;
            public static final int BOTTOM_MOTOR_CAN_ID = 12;

            public static final int CORAL_SENSOR_ID = 0;

            public static final double KP = 0.05;
            public static final double KI = 0.001;
            public static final double KD = 0;
            public static final double FF = 0.0;

            public static final double PID_MIN_OUTPUT = -1.0;
            public static final double PID_MAX_OUTPUT = 1.0;

            public static final double WHEEL_RADIUS = 0.038;
            public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // Meters
            public static final double MOTOR_REDUCTION = 12.0;
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
        public static final String FRONT_CAMERA_NAME = "Arducam_OV2311_USB_Camera"; //
        public static final String RIGHT_CAMERA_NAME = "Right_Arducam_OV9782";
        public static final String LEFT_CAMERA_NAME  = "Left_Arducam_OV9782";

        public static final Transform3d FRONT_CAMERA_OFFSET = new Transform3d(-0.2159, 0, 0, new Rotation3d(0, 0, -Math.PI)); // TODO: Verify that the angle s correct.
        public static final Transform3d RIGHT_CAMERA_OFFSET = new Transform3d(0.219837, 0.1762252, 0.65913, new Rotation3d(0, 0, Math.PI / 2.0 )); 
        public static final Transform3d LEFT_CAMERA_OFFSET = new Transform3d(-0.219837, 0.1760728, 0.65913, new Rotation3d(0, 0, -Math.PI / 2.0)); 

        public static final List<Rotation2d> AVAILABLE_CAMERA_OFFSETS = List.of(
            new Rotation2d(FRONT_CAMERA_OFFSET.getRotation().getMeasureZ())
        );

        public static final double[] VISION_STANDARD_DEVIATION_COEFFICIENTS = { // PLACEHOLDER
            0.01, 0.01, 999999999
        };
        
        public static final Matrix<N3, N1> VISION_STANDARD_DEVIATION = new Matrix<>(N3.instance, N1.instance, VISION_STANDARD_DEVIATION_COEFFICIENTS);

        // Pipeline settings
        public static final int APRIL_TAG_PIPELINE = 0;
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);

        // Odometry Detection Strategy
        public static final PhotonPoseEstimator.PoseStrategy POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        public static final PhotonPoseEstimator.PoseStrategy FALLBACK_STRATEGY = PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
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

        // Time Synchronization
        public static final int TIME_SYNC_PORT_NUMBER = 6000;
        public static final int DEFAULT_NUM_SAMPLES = 10;
    }

    // TODO: Write better, mroe descriptivejavadoc comments for this portion of the code.
    public static class FieldConstants {

        private static final Translation2d FIELD_ORIGIN = new Translation2d(8.775, 4.02);

        // Helper: rotate a Pose2d 180 degrees about the field origin.
        private static Pose2d rotate180(Pose2d bluePose) {
            Translation2d blueTranslation = bluePose.getTranslation();
            // 180° rotation about FIELD_ORIGIN: (2*ox - x, 2*oy - y)
            double redX = 2 * FIELD_ORIGIN.getX() - blueTranslation.getX();
            double redY = 2 * FIELD_ORIGIN.getY() - blueTranslation.getY();
            // Orientation: add π and normalize (Rotation2d takes care of that if needed)
            Rotation2d redRotation = bluePose.getRotation().plus(new Rotation2d(Math.PI));
            return new Pose2d(new Translation2d(redX, redY), redRotation);
        }

        // Helper: Mirror a pose across a horizontal line at a given y-value.
        private static Pose2d mirrorAcrossHorizontal(double yLine, Pose2d bluePose) {
            Translation2d translation = bluePose.getTranslation();
            double mirroredY = 2 * yLine - translation.getY();
            Translation2d newTranslation = new Translation2d(translation.getX(), mirroredY);
            // For a horizontal mirror, the x component stays the same while the y component of any vector is inverted.
            // Here we reflect the rotation by negating its radians.
            double mirroredRadians = -bluePose.getRotation().getRadians();
            Rotation2d newRotation = new Rotation2d(mirroredRadians);
            return new Pose2d(newTranslation, newRotation);
        }

        public static class ReefConstants {

            /* --- BLUE CORAL SCORING POSITIONS --- */
            // Side normals (in radians): 
            // Left: PI, Bottom-Left: 4PI/3, Bottom-Right: 5PI/3, Right: 0, Top-Right: PI/3, Top-Left: 2PI/3

            private static final Translation2d REEF_CENTER = new Translation2d(4.49, 4.02);
            private static final double APOTHEM = 1.212;      // distance from reef center to a side
            private static final double TANGENT_OFFSET = 0.17; // offset along the side

            // LEFT SIDE (normal = PI)
            // Base position for algae removal is at REEF_CENTER + APOTHEM*(cos(PI), sin(PI))
            private static final Translation2d BLUE_LEFT_BASE = 
            new Translation2d(REEF_CENTER.getX() + APOTHEM * Math.cos(Math.PI),
                            REEF_CENTER.getY() + APOTHEM * Math.sin(Math.PI));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_A = new Pose2d(
            new Translation2d(BLUE_LEFT_BASE.getX(), BLUE_LEFT_BASE.getY() + TANGENT_OFFSET),
            new Rotation2d(Math.PI));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_B = new Pose2d(
            new Translation2d(BLUE_LEFT_BASE.getX(), BLUE_LEFT_BASE.getY() - TANGENT_OFFSET),
            new Rotation2d(Math.PI));
            // ALGAE REMOVAL position (midpoint on left side)
            public static final Pose2d BLUE_ALGAE_REMOVAL_POSITION_AB = new Pose2d(
            BLUE_LEFT_BASE, new Rotation2d(Math.PI));

            // BOTTOM-LEFT SIDE (normal = 4PI/3)
            private static final double NORMAL_BL = 4 * Math.PI / 3;
            private static final Translation2d BLUE_BL_BASE = 
            new Translation2d(REEF_CENTER.getX() + APOTHEM * Math.cos(NORMAL_BL),
                            REEF_CENTER.getY() + APOTHEM * Math.sin(NORMAL_BL));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_C = new Pose2d(
            new Translation2d(BLUE_BL_BASE.getX() + TANGENT_OFFSET * Math.sin(NORMAL_BL),
                            BLUE_BL_BASE.getY() + TANGENT_OFFSET * -Math.cos(NORMAL_BL)),
            new Rotation2d(NORMAL_BL));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_D = new Pose2d(
            new Translation2d(BLUE_BL_BASE.getX() - TANGENT_OFFSET * Math.sin(NORMAL_BL),
                            BLUE_BL_BASE.getY() - TANGENT_OFFSET * -Math.cos(NORMAL_BL)),
            new Rotation2d(NORMAL_BL));
            // ALGAE REMOVAL position (midpoint on bottom-left side)
            public static final Pose2d BLUE_ALGAE_REMOVAL_POSITION_CD = new Pose2d(
            BLUE_BL_BASE, new Rotation2d(NORMAL_BL));

            // BOTTOM-RIGHT SIDE (normal = 5PI/3)
            private static final double NORMAL_BR = 5 * Math.PI / 3;
            private static final Translation2d BLUE_BR_BASE = 
                new Translation2d(REEF_CENTER.getX() + APOTHEM * Math.cos(NORMAL_BR),
                                REEF_CENTER.getY() + APOTHEM * Math.sin(NORMAL_BR));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_E = new Pose2d(
                new Translation2d(BLUE_BR_BASE.getX() + TANGENT_OFFSET * Math.sin(NORMAL_BR),
                                BLUE_BR_BASE.getY() + TANGENT_OFFSET * -Math.cos(NORMAL_BR)),
                new Rotation2d(NORMAL_BR));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_F = new Pose2d(
                new Translation2d(BLUE_BR_BASE.getX() - TANGENT_OFFSET * Math.sin(NORMAL_BR),
                                BLUE_BR_BASE.getY() - TANGENT_OFFSET * -Math.cos(NORMAL_BR)),
                new Rotation2d(NORMAL_BR));
                // ALGAE REMOVAL position (midpoint on bottom-left side)
            public static final Pose2d BLUE_ALGAE_REMOVAL_POSITION_EF = new Pose2d(
                BLUE_BR_BASE, new Rotation2d(NORMAL_BR));

            // RIGHT SIDE (normal = 0)
            private static final Translation2d BLUE_RIGHT_BASE = 
                new Translation2d(REEF_CENTER.getX() + APOTHEM * Math.cos(0),
                                REEF_CENTER.getY() + APOTHEM * Math.sin(0));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_G = new Pose2d(
                new Translation2d(BLUE_RIGHT_BASE.getX(), BLUE_RIGHT_BASE.getY() - TANGENT_OFFSET),
                new Rotation2d(0));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_H = new Pose2d(
                new Translation2d(BLUE_RIGHT_BASE.getX(), BLUE_RIGHT_BASE.getY() + TANGENT_OFFSET),
                new Rotation2d(0));
                // ALGAE REMOVAL position (midpoint on bottom-left side)
            public static final Pose2d BLUE_ALGAE_REMOVAL_POSITION_GH = new Pose2d(
                BLUE_RIGHT_BASE, new Rotation2d(0));

            // TOP-RIGHT SIDE (normal = PI/3)
            private static final double NORMAL_TR = Math.PI / 3;
            private static final Translation2d BLUE_TR_BASE = 
                new Translation2d(REEF_CENTER.getX() + APOTHEM * Math.cos(NORMAL_TR),
                                REEF_CENTER.getY() + APOTHEM * Math.sin(NORMAL_TR));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_I = new Pose2d(
                new Translation2d(BLUE_TR_BASE.getX() + TANGENT_OFFSET * Math.sin(NORMAL_TR),
                                BLUE_TR_BASE.getY() + TANGENT_OFFSET * -Math.cos(NORMAL_TR)),
                new Rotation2d(NORMAL_TR));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_J = new Pose2d(
                new Translation2d(BLUE_TR_BASE.getX() - TANGENT_OFFSET * Math.sin(NORMAL_TR),
                                BLUE_TR_BASE.getY() - TANGENT_OFFSET * -Math.cos(NORMAL_TR)),
                new Rotation2d(NORMAL_TR));
                // ALGAE REMOVAL position (midpoint on bottom-left side)
            public static final Pose2d BLUE_ALGAE_REMOVAL_POSITION_IJ = new Pose2d(
                BLUE_TR_BASE, new Rotation2d(NORMAL_TR));

            // TOP-LEFT SIDE (normal = 2PI/3)
            private static final double NORMAL_TL = 2 * Math.PI / 3;
            private static final Translation2d BLUE_TL_BASE = 
            new Translation2d(REEF_CENTER.getX() + APOTHEM * Math.cos(NORMAL_TL),
                            REEF_CENTER.getY() + APOTHEM * Math.sin(NORMAL_TL));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_K = new Pose2d(
            new Translation2d(BLUE_TL_BASE.getX() + TANGENT_OFFSET * Math.sin(NORMAL_TL),
                            BLUE_TL_BASE.getY() + TANGENT_OFFSET * -Math.cos(NORMAL_TL)),
            new Rotation2d(NORMAL_TL));
            public static final Pose2d BLUE_CORAL_SCORE_POSITION_L = new Pose2d(
            new Translation2d(BLUE_TL_BASE.getX() - TANGENT_OFFSET * Math.sin(NORMAL_TL),
                            BLUE_TL_BASE.getY() - TANGENT_OFFSET * -Math.cos(NORMAL_TL)),
            new Rotation2d(NORMAL_TL));
            // ALGAE REMOVAL position (midpoint on top-left side)
            public static final Pose2d BLUE_ALGAE_REMOVAL_POSITION_KL = new Pose2d(
            BLUE_TL_BASE, new Rotation2d(NORMAL_TL));

            /* --- RED POSITIONS (rotated 180° about FIELD_ORIGIN) --- */
            // Coral scoring positions
            public static final Pose2d RED_CORAL_SCORE_POSITION_A = rotate180(BLUE_CORAL_SCORE_POSITION_A);
            public static final Pose2d RED_CORAL_SCORE_POSITION_B = rotate180(BLUE_CORAL_SCORE_POSITION_B);
            public static final Pose2d RED_CORAL_SCORE_POSITION_C = rotate180(BLUE_CORAL_SCORE_POSITION_C);
            public static final Pose2d RED_CORAL_SCORE_POSITION_D = rotate180(BLUE_CORAL_SCORE_POSITION_D);
            public static final Pose2d RED_CORAL_SCORE_POSITION_E = rotate180(BLUE_CORAL_SCORE_POSITION_E);
            public static final Pose2d RED_CORAL_SCORE_POSITION_F = rotate180(BLUE_CORAL_SCORE_POSITION_F);
            public static final Pose2d RED_CORAL_SCORE_POSITION_G = rotate180(BLUE_CORAL_SCORE_POSITION_G);
            public static final Pose2d RED_CORAL_SCORE_POSITION_H = rotate180(BLUE_CORAL_SCORE_POSITION_H);
            public static final Pose2d RED_CORAL_SCORE_POSITION_I = rotate180(BLUE_CORAL_SCORE_POSITION_I);
            public static final Pose2d RED_CORAL_SCORE_POSITION_J = rotate180(BLUE_CORAL_SCORE_POSITION_J);
            public static final Pose2d RED_CORAL_SCORE_POSITION_K = rotate180(BLUE_CORAL_SCORE_POSITION_K);
            public static final Pose2d RED_CORAL_SCORE_POSITION_L = rotate180(BLUE_CORAL_SCORE_POSITION_L);
            // Algae removal positions
            public static final Pose2d RED_ALGAE_REMOVAL_POSITION_AB = rotate180(BLUE_ALGAE_REMOVAL_POSITION_AB);
            public static final Pose2d RED_ALGAE_REMOVAL_POSITION_CD = rotate180(BLUE_ALGAE_REMOVAL_POSITION_CD);
            public static final Pose2d RED_ALGAE_REMOVAL_POSITION_EF = rotate180(BLUE_ALGAE_REMOVAL_POSITION_EF);
            public static final Pose2d RED_ALGAE_REMOVAL_POSITION_GH = rotate180(BLUE_ALGAE_REMOVAL_POSITION_GH);
            public static final Pose2d RED_ALGAE_REMOVAL_POSITION_IJ = rotate180(BLUE_ALGAE_REMOVAL_POSITION_IJ);
            public static final Pose2d RED_ALGAE_REMOVAL_POSITION_KL = rotate180(BLUE_ALGAE_REMOVAL_POSITION_KL);

        }

        public static class CoralStationConstants {
            
            // === Coral Station Intake Positions ===
            private static final double CORAL_INTAKE_SPACING = 0.2032;

            // Convert to a field–relative angle: add π/2 since the wall is vertical.
            private static final Rotation2d FIELD_INTAKE_ANGLE = new Rotation2d(3.76971235639);

            // Base pose for BLUE LEFT intake positions.
            // Index 0 is given as (1.70244, 7.57545) with the computed intake angle.
            private static final Pose2d BLUE_INTAKE_LEFT_BASE = new Pose2d(
                    new Translation2d(1.70298, 7.57847), FIELD_INTAKE_ANGLE);

            // Lists for the coral station intake poses.
            // The human player’s list is from indices 0 to 8 (left-to-right from the driver perspective).
            public static final List<Pose2d> BLUE_CORAL_INTAKE_LEFT;
            public static final List<Pose2d> BLUE_CORAL_INTAKE_RIGHT;
            public static final List<Pose2d> RED_CORAL_INTAKE_LEFT;
            public static final List<Pose2d> RED_CORAL_INTAKE_RIGHT;

            /**
             * Create the list of intaking positions on the fly given preceding conditions. These arrays are immutable, 
             * and cannot be changed.
             */
            static {

                List<Pose2d> blueCoralIntakeLeft = new ArrayList<>();
                List<Pose2d> blueCoralIntakeRight = new ArrayList<>();
                List<Pose2d> redCoralIntakeLeft = new ArrayList<>();
                List<Pose2d> redCoralIntakeRight = new ArrayList<>();
                // Generate BLUE LEFT intake positions using Transform2d.
                // Each position is obtained by moving forward (local x) by i * spacing.
                for (int i = 0; i < 9; i++) {
                    Pose2d pose = BLUE_INTAKE_LEFT_BASE.transformBy(
                            new Transform2d(new Translation2d(i * CORAL_INTAKE_SPACING, 0), new Rotation2d(0.5 * Math.PI)));
                            blueCoralIntakeLeft.add(pose);
                }
                // Generate BLUE RIGHT intake positions by mirroring across the horizontal line at y = FIELD_ORIGIN.getY()
                // Reverse the order so that the human player's indices run left-to-right.
                for (int i = blueCoralIntakeLeft.size() - 1; i >= 0; i--) {
                    blueCoralIntakeRight.add(mirrorAcrossHorizontal(FIELD_ORIGIN.getY(), blueCoralIntakeLeft.get(i)));
                }
                // Generate corresponding RED positions by rotating the blue positions 180° about FIELD_ORIGIN.
                for (Pose2d pose : blueCoralIntakeLeft) {
                    redCoralIntakeLeft.add(rotate180(pose));
                }
                for (Pose2d pose : blueCoralIntakeRight) {
                    redCoralIntakeRight.add(rotate180(pose));
                }

                // Save the arrays in an immutable way.
                BLUE_CORAL_INTAKE_LEFT = Collections.unmodifiableList(blueCoralIntakeLeft);
                BLUE_CORAL_INTAKE_RIGHT = Collections.unmodifiableList(blueCoralIntakeRight);
                RED_CORAL_INTAKE_LEFT = Collections.unmodifiableList(redCoralIntakeLeft);
                RED_CORAL_INTAKE_RIGHT = Collections.unmodifiableList(redCoralIntakeRight);
            }

        }

        public static class StartingPoseConstants {

            public static final Pose2d BLUE_LEFT_STARTING_POSE = new Pose2d(new Translation2d(7.247,6.16),new Rotation2d());
            public static final Pose2d BLUE_CENTER_STARTING_POSE = new Pose2d(new Translation2d(7.247,4.19),new Rotation2d());
            public static final Pose2d BLUE_RIGHT_STARTING_POSE = new Pose2d(new Translation2d(7.247,1.88),new Rotation2d());

            public static final Pose2d RED_LEFT_STARTING_POSE = rotate180(BLUE_LEFT_STARTING_POSE);
            public static final Pose2d RED_CENTER_STARTING_POSE = rotate180(BLUE_CENTER_STARTING_POSE);
            public static final Pose2d RED_RIGHT_STARTING_POSE = rotate180(BLUE_RIGHT_STARTING_POSE);

            // The offset if starting with a tush push (middle of the tape, rather than edge)
            public static final Transform2d TUSH_PUSH_STARTING_TRANSFORM = new Transform2d(-0.0254,0.0,new Rotation2d());

            // The offset tush push will move the robot. This is relative to the transformed starting pose.
            public static final Transform2d TUSH_PUSH_TRANSFORM = new Transform2d(0.10,0.0,new Rotation2d());

        }

    }
}

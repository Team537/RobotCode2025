// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Position offsets, used to determine the orientation of the driver
    public static final Rotation2d BLUE_ALLIANCE_OFFSET = new Rotation2d(-0.5 * Math.PI);
    public static final Rotation2d RED_ALLIANCE_OFFSET = new Rotation2d(0.5 * Math.PI);
    public static final Rotation2d DEMO_ALLIANCE_OFFSET = new Rotation2d(-0.5 * Math.PI);

    //Controller Constants
    public static final double LINEAR_INPUT_CURVE_POWER = 2.5;
    public static final double ROTATION_INPUT_CURVE_POWER = 2.5;
    public static final double THROTTLE_LINEAR_MIN_SPEED = 2.0; //Meters per second
    public static final double THROTTLE_LINEAR_MAX_SPEED = DriveConstants.LINEAR_MAX_SPEED; //Meters per second
    public static final double THROTTLE_ROTATIONAL_MIN_SPEED = 2.5; //Radians per second
    public static final double THROTTLE_ROTATIONAL_MAX_SPEED = DriveConstants.ROTATIONAL_MAX_SPEED; //Radians per second

    public static final double XBOX_CONTROLLER_JOYSTICK_DEADMAND_RADIUS = 0.01;
    public static final double XBOX_CONTROLLER_TARGET_MIN_RADIUS = 1.0; //Meters
    public static final double XBOX_CONTROLLER_TARGET_MAX_RADIUS = 5.0; //Meters
    public static final double XBOX_CONTROLLER_ROTATIONAL_TARGET_ACTIVATION_ZONE = 0.8;
    public static final double XBOX_CONTROLLER_ROTATIONAL_TARGET_DEACTIVATION_ZONE = 0.7;

  }

  //Constants used for items relating to the drivetrain
  public static class DriveConstants {

    //Controller Constants
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
    public static final double DRIVING_MOTOR_FREE_SPEED = 94.6; //Revolutions per second
    public static final double WHEEL_DIAMETER = 0.06677; //Meters
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //Meters
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = 4.714;
    public static final double DRIVE_WHEEL_FREE_SPEED = (DRIVING_MOTOR_FREE_SPEED * WHEEL_CIRCUMFERENCE) / DRIVING_MOTOR_REDUCTION; // Meters / Second

    public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

    public static final boolean DRIVING_ENCODER_INVERTED = false;

    //PIDs for the Driving Motor
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

    //PIDs for the turning Motor
    public static final double TURNING_KP = 1;
    public static final double TURNING_KI = 0;
    public static final double TURNING_KD = 0;
    public static final double TURNING_FF = 0;
    public static final double TURNING_PID_MIN_OUTPUT = -1.0;
    public static final double TURNING_PID_MAX_OUTPUT = 1.0;

    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps

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
    public static final double LINEAR_MAX_SPEED = 4.8; //Meters per second
    public static final double ROTATIONAL_MAX_SPEED = 6.0; //Radians per second
    public static final double LINEAR_MAX_ACCELERATION = 5.0; //Meters per second squared
    public static final double ROTATIONAL_MAX_ACCELERATION = 6.4; //Radians per second squared

    // Maxmimum delta time for driving, prevents too fast accelerations when lag occurs
    public static final double MAX_DELTA_TIME_RATE_LIMIT = 0.1;//1.0; //seconds

    // The drivetrain size and kinematics

    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(16);
    
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(16);
    
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

  }
  public static class NarwhalConstants {
    public static class NarwhalIntakeOuttakeConstants {
      public static final int INTAKE_OUTTAKE_MOTOR_CAN_ID = 12; // TODO: Replace this placeholder with the actual id
      public static final int INTAKE_OUTTAKE_MOTOR_CURRENT_LIMIT = 20;
      public static final double INTAKE_MOTOR_PERCENTAGE = 0.35; // between -1.0 and 1.0
      public static final double OUTTAKE_MOTOR_PERCENTAGE = -0.35; // between -1.0 and 1.0
      
      public static final double POSITION_PID_P = 0.7;
      public static final double POSITION_PID_I = 0;
      public static final double POSITION_PID_D = 0.2;
    }
  }
  
  public static class VisionConstants {

    // Pipeline settings
    public static final int APRIL_TAG_PIPELINE = 0;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  }
}
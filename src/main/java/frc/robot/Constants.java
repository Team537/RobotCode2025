// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class VisionConstants {

    // Pipeline settings
    public static final int APRIL_TAG_PIPELINE = 0;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  }

  public static final class FalconMotorConstants {
    // these are the max motor speed of a FALCON motor
    public static final int FREE_SPEED_RPM = 6380;
    public static final double FREE_SPEED_RPS = 106.33; //actual value is 106 and 1/3
  }
  public static final class KrackenX60MotorConstants {
    // these are the max motor speed of a KRACKEN_X60 motor
    public static final int FREE_SPEED_RPM = 5800;
    public static final double FREE_SPEED_RPS = 96.67; //actual value is 96 and 2/3
  }

  public static final class TalonMotionMagicConstants {
    public static final double KV = 0.13;
    public static final double KA = 0.00;
    public static final double KG = 0.00;

    public static final double KP = 4;
    public static final double KI = 0;
    public static final double KD = 0.1;

    //units in RPS
    public static final double CRUISE_VELOCITY_UP = 80;
    public static final double CRUISE_VELOCITY_DOWN = 50;
    public static final double ACCELERATION = 60;
    public static final double JERK = 100;
    
}
  public static final class TalonPIDConstants {
    public static final double KP = 1;
    public static final double KI = 1;
    public static final double KD = 0.01;
  }
  public static final class TalonVelocityConstants {        
    public static final double KS = 0.05;
    public static final double KV = 0.12;
    public static final double KP = 0.11;
  }  
}

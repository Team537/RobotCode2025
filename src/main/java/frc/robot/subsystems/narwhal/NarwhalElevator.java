// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.narwhal;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NarwhalConstants.NarwhalElevatorConstants;
import frc.robot.util.NarwhalElevatorState;


public class NarwhalElevator extends SubsystemBase {
    public NarwhalElevatorState currentState;
    private double currentTargetPosition;

    private final SparkMax leadElevator;
    private final SparkMaxConfig leadElevatorConfig;
    private final SparkClosedLoopController leadElevatorPID;
    
    private final SparkMax followerElevator;
    private final SparkMaxConfig followerElevatorConfig;

    public NarwhalElevator(){
        currentTargetPosition = 0.0;

        // Configurations for the Spark Max that controls the lead elevator motor
        leadElevatorConfig = new SparkMaxConfig();
        leadElevatorConfig // general configs
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(NarwhalElevatorConstants.ELEVATOR_LEAD_MOTOR_CURRENT_LIMIT)
            .inverted(NarwhalElevatorConstants.MOTOR_INVERTED);
        leadElevatorConfig.encoder // encoder configs
            .positionConversionFactor(NarwhalElevatorConstants.ENCODER_FACTOR)
            .velocityConversionFactor(NarwhalElevatorConstants.ENCODER_FACTOR/60.0); // dividing by 60 accounts for RPM to Radians/Sec
        leadElevatorConfig.closedLoop // pid 
            .outputRange(NarwhalElevatorConstants.ELEVATOR_MIN_OUTPUT, NarwhalElevatorConstants.ELEVATOR_MAX_OUTPUT)
            .pid(
                NarwhalElevatorConstants.ELEVATOR_KP,
                NarwhalElevatorConstants.ELEVATOR_KI,
                NarwhalElevatorConstants.ELEVATOR_KD
            );

        // creating the spark max controller
        leadElevator = new SparkMax(NarwhalElevatorConstants.ELEVATOR_LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
        leadElevator.configure(leadElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Configurations for the follower Spark Max that controls the follower elevator motor
        followerElevatorConfig = new SparkMaxConfig();
        followerElevatorConfig // general configs
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(NarwhalElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_CURRENT_LIMIT)
            .follow(NarwhalElevatorConstants.ELEVATOR_LEAD_MOTOR_CAN_ID);

        // creating the spark max controller
        followerElevator = new SparkMax(NarwhalElevatorConstants.ELEVATOR_FOLLOWER_CAN_ID, MotorType.kBrushless);
        followerElevator.configure(followerElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // creating the PID controller
        leadElevatorPID = leadElevator.getClosedLoopController();
        
        //state
        currentState = NarwhalElevatorState.BOTTOM;
    }

    
    /**
     * Function to go to a specific height in meters. Measured from the zero position. 
     * @param meters positive - 0.0 is bottomed out. values are automatically clamped between MIN_HEIGHT_METERS and MAX_HEIGHT_METERS
     */
    public void setHeight(double meters){
        double target_height = Math.min(Math.max(meters, NarwhalElevatorConstants.MIN_HEIGHT_METERS), NarwhalElevatorConstants.MAX_HEIGHT_METERS); // Clamps the height to [MAX_HEIGHT_METERS, MIN_HEIGHT_METERS]
        leadElevatorPID.setReference(target_height, ControlType.kPosition); // Runs using percent output of duty cycle
        currentState = NarwhalElevatorState.CUSTOM;

        currentTargetPosition = target_height;
    }

    /**
     * Goes to the height set as the elevator's minimum (MIN_HEIGHT_METERS)
     */
    public void goToMinimumHeight(){
        setHeight(NarwhalElevatorConstants.MIN_HEIGHT_METERS);
        currentState = NarwhalElevatorState.BOTTOM;
    }
    
    /**
     * Goes to Intaking Height defined by {@link NarwhalElevatorConstants#INTAKE_ELEVATOR_HEIGHT_METERS}
     */
    public void goToIntakeHeight(){
        setHeight(NarwhalElevatorConstants.INTAKE_ELEVATOR_HEIGHT_METERS);
        currentState = NarwhalElevatorState.INTAKE;
    }

    /**
     * Goes to the height for scoring L1 defined by {@link NarwhalElevatorConstants#L1_ELEVATOR_HEIGHT}
     */
    public void goToScoreHeightL1(){
        setHeight(NarwhalElevatorConstants.L1_ELEVATOR_HEIGHT);
        currentState = NarwhalElevatorState.L1;
    }

    /**
     * Goes to the height for scoring L2 defined by {@link NarwhalElevatorConstants#L2_ELEVATOR_HEIGHT}
     */
    public void goToScoreHeightL2(){
        setHeight(NarwhalElevatorConstants.L2_ELEVATOR_HEIGHT);
        currentState = NarwhalElevatorState.L2;
    }

    /**
     * Goes to the height for scoring L3 defined by {@link NarwhalElevatorConstants#L3_ELEVATOR_HEIGHT}
     */
    public void goToScoreHeightL3(){
        setHeight(NarwhalElevatorConstants.L3_ELEVATOR_HEIGHT);
        currentState = NarwhalElevatorState.L3;
    }

    /**
     * Goes to the height for scoring L4 defined by {@link NarwhalElevatorConstants#L4_ELEVATOR_HEIGHT} :)
     */
    public void goToScoreHeightL4(){
        setHeight(NarwhalElevatorConstants.L4_ELEVATOR_HEIGHT);
        currentState = NarwhalElevatorState.L4;
    }

    /**
     * Checks if the elevator is at the target position. Note: this method does not account for velocity and uses a tolerance.
     * 
     * @return Returns true if the elevator is at the target position
     */
    public boolean isAtTargetPosition(){
        return Math.abs(leadElevator.getEncoder().getPosition() - currentTargetPosition) < NarwhalElevatorConstants.ELEVATOR_POSITION_TOLERANCE;
    }

    @Override
    public void periodic(){
        // This method will be called once per scheduler run during simulation
    }
}
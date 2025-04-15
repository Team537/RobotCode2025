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
import frc.robot.util.upper_assembly.ScoringHeight;


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
     * @param height positive - 0.0 is bottomed out. values are automatically clamped between MIN_HEIGHT_METERS and MAX_HEIGHT_METERS
     */
    public void setHeight(double height){
        double target_height = Math.min(Math.max(height, NarwhalElevatorConstants.MIN_HEIGHT_METERS), NarwhalElevatorConstants.MAX_HEIGHT_METERS); // Clamps the height to [MAX_HEIGHT_METERS, MIN_HEIGHT_METERS]
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
     * Goes to the specified scoring height.
     *
     * @param height The target scoring height.
     */
    public void goToScoreHeight(ScoringHeight height) {
        switch (height) {
            case L1:
                setHeight(NarwhalElevatorConstants.L1_ELEVATOR_HEIGHT);
                currentState = NarwhalElevatorState.L1;
                break;
            case L2:
                setHeight(NarwhalElevatorConstants.L2_ELEVATOR_HEIGHT);
                currentState = NarwhalElevatorState.L2;
                break;
            case L3:
                setHeight(NarwhalElevatorConstants.L3_ELEVATOR_HEIGHT);
                currentState = NarwhalElevatorState.L3;
                break;
            case L4:
                setHeight(NarwhalElevatorConstants.L4_ELEVATOR_HEIGHT);
                currentState = NarwhalElevatorState.L4;
                break;
            default:
                throw new IllegalArgumentException("Unknown scoring height: " + height);
        }
        
    }

    /**
     * Goes to the algae descore position
     * @param isTopRow True if the algae is in the top row, false if it is in the bottom row
     * @param isDown True if the manipulator should be pressing down on the algae, false if it should just be above the algae
     */
    public void goToAlgaeDescorePosition(boolean isTopRow, boolean isDown){
        double targetHeight;

        // Set the target height based on the row
        if (isTopRow){
            targetHeight = NarwhalElevatorConstants.ALGAE_DESCORE_HEIGHT_METERS_TOP_ROW;
        } else {
            targetHeight = NarwhalElevatorConstants.ALGAE_DESCORE_HEIGHT_METERS_BOTTOM_ROW;
        }

        // Apply an offset if the manipulator should be pressing down on the algae
        if (isDown){
            targetHeight += NarwhalElevatorConstants.ALGAE_DESCORE_HEIGHT_METERS_DOWN_OFFSET;
        }

        setHeight(targetHeight);

        currentState = NarwhalElevatorState.ALGAE_DESCORE;
    }

    /**
     * Checks if the elevator is at the target position. Note: this method does not account for velocity and uses a tolerance.
     * 
     * @return Returns true if the elevator is at the target position
     */
    public boolean isAtTargetPosition(){
        return Math.abs(leadElevator.getEncoder().getPosition() - currentTargetPosition) < NarwhalElevatorConstants.ELEVATOR_POSITION_TOLERANCE;
    }

    /**
     * Gets the current state of the elevtaor
     */
    public NarwhalElevatorState getCurrentState(){
        return this.currentState;
    }

    @Override
    public void periodic(){
        // This method will be called once per scheduler run during simulation
    }
}
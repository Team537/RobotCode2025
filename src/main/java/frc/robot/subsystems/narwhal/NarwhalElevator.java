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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NarwhalConstants;
import frc.robot.Constants.NarwhalConstants.NarwhalElevatorConstants;
import edu.wpi.first.wpilibj.XboxController;


public class NarwhalElevator extends SubsystemBase {
    public NarwhalElevatorState currentState;

    /** Inputs and outputs meters */
    private final SparkMax leadElevator;
    private final SparkMaxConfig leadElevatorConfig;
    private final SparkClosedLoopController leadElevatorPID;

    private final SparkMax followerElevator;
    private final SparkMaxConfig followerElevatorConfig;

    public NarwhalElevator(){
        // lead
        leadElevatorConfig = new SparkMaxConfig();
        leadElevatorConfig // general configs
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(NarwhalElevatorConstants.ELEVATOR_LEAD_MOTOR_CURRENT_LIMIT);
        leadElevatorConfig.encoder // encoder configs
            .positionConversionFactor(NarwhalElevatorConstants.ROTATIONS_TO_METERS)
            .velocityConversionFactor(NarwhalElevatorConstants.ROTATIONS_TO_METERS/60.0); // dividing by 60 accounts for RPM to Radians/Sec
        leadElevatorConfig.closedLoop // pid configs
            .pid(
                NarwhalElevatorConstants.ELEVATOR_HEIGHT_PID_P,
                NarwhalElevatorConstants.ELEVATOR_HEIGHT_PID_I,
                NarwhalElevatorConstants.ELEVATOR_HEIGHT_PID_D
            );

        leadElevator = new SparkMax(NarwhalElevatorConstants.ELEVATOR_LEAD_MOTOR_CAN_ID, MotorType.kBrushless);
        leadElevator.configure(leadElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leadElevatorPID = leadElevator.getClosedLoopController();
        
        // follower
        followerElevatorConfig = new SparkMaxConfig();
        followerElevatorConfig
            .idleMode(IdleMode.kBrake)
            .follow(NarwhalElevatorConstants.ELEVATOR_LEAD_MOTOR_CAN_ID)
            .smartCurrentLimit(NarwhalElevatorConstants.ELEVATOR_FOLLOWER_MOTOR_CURRENT_LIMIT);

        followerElevator = new SparkMax(NarwhalElevatorConstants.ELEVATOR_FOLLOWER_CAN_ID, MotorType.kBrushless);
        followerElevator.configure(followerElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        currentState = NarwhalElevatorState.BOTTOM;
    }

    
    /**
     * Function to go to a specific height in meters. Measured from the zero position. 
     * @param meters positive - 0.0 is bottomed out
     */
    public void setHeight(double meters){
        double target_height = Math.min(Math.max(meters, NarwhalElevatorConstants.MIN_HEIGHT_METERS), NarwhalElevatorConstants.MAX_HEIGHT_METERS); // Clamps the height to [MAX_HEIGHT_METERS, MIN_HEIGHT_METERS]
        leadElevatorPID.setReference(target_height, ControlType.kPosition); // Runs using percent output of duty cycle
        currentState = NarwhalElevatorState.CUSTOM;
    }

    /**
     * Goes to Intaking Height
     */
    public void intake(){
        setHeight(NarwhalElevatorConstants.INTAKE_HEIGHT_METERS);
        currentState = NarwhalElevatorState.INTAKE;
    }

    public void bottom(){
        setHeight(NarwhalElevatorConstants.MIN_HEIGHT_METERS);
        currentState = NarwhalElevatorState.BOTTOM;
    }

    /**
     * Goes to L1 Height
     */
    public void L1(){
        setHeight(NarwhalElevatorConstants.L1_METERS);
        currentState = NarwhalElevatorState.L1;
    }

    /**
     * Goes to L2 Height
     */
    public void L2(){
        setHeight(NarwhalElevatorConstants.L2_METERS);
        currentState = NarwhalElevatorState.L2;
    }

    /**
     * Goes to L3 Height
     */
    public void L3(){
        setHeight(NarwhalElevatorConstants.L3_METERS);
        currentState = NarwhalElevatorState.L3;
    }

    /**
     * Goes to L4 Height :)
     */
    public void L4(){
        setHeight(NarwhalElevatorConstants.L4_METERS);
        currentState = NarwhalElevatorState.L4;
    }

    public void runXBoxController(XboxController xboxController){
        if(xboxController.getRightBumperButton()){
            intake();
        }
        else if(xboxController.getBackButton()){
            L1();
        }
        else if(xboxController.getYButton()){
            L4();
        }
        else if(xboxController.getBButton()){
            L3();
        }
        else if(xboxController.getAButton()){
            L2();
        }
    }
}
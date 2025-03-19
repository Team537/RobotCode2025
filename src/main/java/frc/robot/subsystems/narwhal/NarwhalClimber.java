// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.narwhal;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.NarwhalConstants.NarwhalClimberConstants;
import frc.robot.util.upper_assembly.narwhal.NarwhalClimberState;

/**
 * <h2> NarwhalWrist </h2>
 * The {@code NarwhalWrist} class is a class that represents the Narwhal's wrist mechanism.
 * It allows for game elements to be rotated around the wrist's base, fueling complex movement and design.
 * <hr>
 * @author Patrick Wang
 * @since v1.2.0
 */
public class NarwhalClimber extends SubsystemBase {
    public NarwhalClimberState currentState;
    
    private final SparkMax climber;
    private final SparkMaxConfig climberConfig;
    private final SparkClosedLoopController climberPID;
    
    /**
     * Creates a new instance of the NarwhalClimber class, setting up all necessary hardware in the process.
     */
    public NarwhalClimber() {

        // Create a new spark max to control the climber.
        climberConfig = new SparkMaxConfig();
        
        // Configure the climber motor settings.
        climberConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(NarwhalClimberConstants.IS_CLIMBER_INVERTED);

        // Adjust the encoder settings.
        climberConfig.encoder
            .positionConversionFactor(1.0/NarwhalClimberConstants.CLIMBER_ANGLE_TO_MOTOR_ANGLE);

        // Update motor PID values.
        climberConfig.closedLoop
            .pidf(
                NarwhalClimberConstants.PID_P,
                NarwhalClimberConstants.PID_I,
                NarwhalClimberConstants.PID_D,
                NarwhalClimberConstants.PID_F)
            .outputRange(
                NarwhalClimberConstants.CLIMBER_PID_MIN_OUTPUT, 
                NarwhalClimberConstants.CLIMBER_PID_MAX_OUTPUT
            );
        
        // Creating the motor & setting the configs
        climber = new SparkMax(NarwhalClimberConstants.CLIMBER_CAN_ID, MotorType.kBrushless);
        climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climberPID = climber.getClosedLoopController();
        currentState = NarwhalClimberState.STARTING;
    }

    /**
     * Function to move the climber piece to a target angle (relative to the world) & sets the current state to CUSTOM.
     * 
     * @param targetAngle Rotation2d target position
     */
    public void setCurrentMotorAngle(Rotation2d targetAngle){
        double targetAngleRotations = targetAngle.getRotations();
        // updates the PID to the target value.
        climberPID.setReference(targetAngleRotations, ControlType.kPosition);
        currentState = NarwhalClimberState.CUSTOM;
    }

    /**
     * Set the climber to the deploy angle stored in constants
     */
    public void goToDeploy() {
        setCurrentMotorAngle(NarwhalClimberConstants.DEPLOYED_WINCH_ROTATIONS);
        currentState = NarwhalClimberState.DEPLOYING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the climber to the target angle for climbing stored in constants
     */
    public void climb() {
        setCurrentMotorAngle(NarwhalClimberConstants.CLIMB_WINCH_ROTATIONS);
        currentState = NarwhalClimberState.CLIMBING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Check if the climber is at the Climbing angle
     */
    public boolean isAtClimbAngle(){
        double current_position = climber.getEncoder().getPosition();
        return Math.abs(current_position - NarwhalClimberConstants.CLIMB_WINCH_ROTATIONS.getRotations()) < NarwhalClimberConstants.CLIMBER_ANGLE_TOLERANCE.getRotations();
    }

    /**
     * Check if the climber is at the Deploy angle
     */
    public boolean isAtDeployAngle(){
        double current_position = climber.getEncoder().getPosition();
        return Math.abs(current_position - NarwhalClimberConstants.DEPLOYED_WINCH_ROTATIONS.getRotations()) < NarwhalClimberConstants.CLIMBER_ANGLE_TOLERANCE.getRotations();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

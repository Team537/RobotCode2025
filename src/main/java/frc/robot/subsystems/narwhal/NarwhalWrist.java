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
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NarwhalConstants.NarwhalWristConstants;

/**
 * <h2> NarwhalWrist </h2>
 * The {@code NarwhalWrist} class is a class that represents the Narwhal's wrist mechanism.
 * It allows for game elements to be rotated around the wrist's base, fueling complex movement and design.
 * <hr>
 * @author Patrick Wang
 * @since v1.2.0
 */
public class NarwhalWrist extends SubsystemBase {
    public NarwhalWristState currentState;
    
    private final SparkMax wrist;
    private final SparkMaxConfig wristConfig;
    private final SparkClosedLoopController wristMotorPIDController;
    
    /**
     * Create a new instance of the NarwhalWrist class, setting up necessary hardware in the process.
     */
    public NarwhalWrist() {

        // Create a new spark max to control the wrist.
        wristConfig = new SparkMaxConfig();

        // Configure the wrist motor settings.
        wristConfig 
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.NarwhalConstants.NarwhalWristConstants.WRIST_MOTOR_CURRENT_LIMIT)
            .inverted(true);

        // Update motor PID values.
        wristConfig.closedLoop 
            .pid(
                Constants.NarwhalConstants.NarwhalWristConstants.POSITION_PID_P, 
                Constants.NarwhalConstants.NarwhalWristConstants.POSITION_PID_I, 
                Constants.NarwhalConstants.NarwhalWristConstants.POSITION_PID_D
            )
            .outputRange(-1, 1)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder); // uses external encoder

        // Update encoder settings.
        // NOTE FOR THE ENCODER: WHEN VIEWED FROM THE RIGHT, THE ANGLE OF THE WRIST IS BASED ON A UNIT CIRCLE WITH 0 DEGREES POINTING STRAIGHT UP
        wristConfig.absoluteEncoder
            .inverted(false)
            .positionConversionFactor(Constants.NarwhalConstants.NarwhalWristConstants.ROTATIONS_TO_RADIANS)
            .velocityConversionFactor(Constants.NarwhalConstants.NarwhalWristConstants.ROTATIONS_TO_RADIANS/60.0) // dividing by 60 accounts for RPM to Radians/Sec
            .zeroOffset(Constants.NarwhalConstants.NarwhalWristConstants.WRIST_OFFSET) // Down position should be "0" here, and -PI rads for zero centered
            .zeroCentered(true);

        // Create the spark max controller with the above configured settings.
        wrist = new SparkMax(Constants.NarwhalConstants.NarwhalWristConstants.WRIST_MOTOR_CAN_ID, MotorType.kBrushless);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wristMotorPIDController = wrist.getClosedLoopController();
        currentState = NarwhalWristState.STOPPED;
    }

    public boolean readyToIntake(){
        double current_position = wrist.getAbsoluteEncoder().getPosition();
        return Math.abs(current_position - NarwhalWristConstants.INTAKE_ANGLE.getRadians()) < NarwhalWristConstants.WRIST_ANGLE_TOLERANCE;
    }

    /**
     * Method to move the motor to a target angle & sets the current state to CUSTOM.
     * 
     * @param percent Percentage between -1.0 and 1.0 (negative values reverse direction)
     */
    public void setCurrentMotorAngle(Rotation2d targetAngle){
        double targetAngleRadians = targetAngle.getRadians();
        wristMotorPIDController.setReference(targetAngleRadians, ControlType.kPosition);
        currentState = NarwhalWristState.CUSTOM;
    }

    /**
     * Set the wrist motor to the intake angle (defined in constants) & update status.
     */
    public void goToIntakeAngle() {
        
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        setCurrentMotorAngle(Constants.NarwhalConstants.NarwhalWristConstants.INTAKE_ANGLE);
        currentState = NarwhalWristState.INTAKING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the wrist motor to the outtake angle (defined in constants) & update status.
     */
    public void goToOuttakeAngle() {
        setCurrentMotorAngle(Constants.NarwhalConstants.NarwhalWristConstants.OUTTAKE_ANGLE);
        currentState = NarwhalWristState.OUTTAKING; // must be after the set function because the set function will default to CUSTOM state
    }
    
    /**
     * Set the wrist motor to the algae descore angle (defined in constants) & update status.
     */
    public void goToAlgaeAngle() {
        setCurrentMotorAngle(Constants.NarwhalConstants.NarwhalWristConstants.ALGAE_ANGLE);
        currentState = NarwhalWristState.ALGAE; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the wrist motor to 0 percent output (assumes idle-mode is breaking).
     */
    public void stop() {
        wrist.set(0); // TODO: test that this works & properly disables the PID
        currentState = NarwhalWristState.STOPPED; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Uses a PID to actively hold the position of the motor when this function is called.
     */
    public void hold() {
        double current_position = wrist.getAbsoluteEncoder().getPosition();
        setCurrentMotorAngle(Rotation2d.fromRadians(current_position));
        currentState = NarwhalWristState.CUSTOM; // redundant but helps with readability
    }

    /**
     * Returns the current angle of the wrist, as a {@link Rotation2d}.
     * 
     * @return The current angle of the wrist, as a {@link Rotation2d}.
     */
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(wrist.getAbsoluteEncoder().getPosition());
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

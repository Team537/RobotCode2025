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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants;
import frc.robot.util.upper_assembly.narwhal.NarwhalIntakeOuttakeState;

/**
 * <h2> NarwhalIntakeOuttake </h2>
 * The {@code NarwhalIntakeOuttake} class is a class that represents the Narwhal's intake and outtake.
 * It allows for the intake/outtake motors to be efficiently controlled, allowing for game elements to 
 * be easily and reliably controlled.
 * <hr>
 * @author Patrick Wang
 * @since v1.2.0
 */
public class NarwhalIntakeOuttake extends SubsystemBase {
    public NarwhalIntakeOuttakeState currentState;
    
    /** Limit switch to keep track of if a coral has been intaked */
    private final DigitalInput intakeSensor;

    private final SparkMax intakeOuttakeMotor;
    private final SparkMaxConfig intakeOuttakeMotorConfig;
    private final SparkClosedLoopController intakeOuttakeMotorPIDController;
    
    /**
     * Creates a new instance of the NarwhalIntakeOuttake class, setting up all necessary hardware in the process.
     */
    public NarwhalIntakeOuttake() {
        intakeSensor = new DigitalInput(NarwhalIntakeOuttakeConstants.CORAL_LIMIT_SWITCH_PORT);

        // Create a new spark max to control the intake.
        intakeOuttakeMotorConfig = new SparkMaxConfig();

        // Configure the intake motor settings.
        intakeOuttakeMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(NarwhalIntakeOuttakeConstants.INTAKE_OUTTAKE_MOTOR_CURRENT_LIMIT);
        
        // Update motor PID values.
        intakeOuttakeMotorConfig.closedLoop
                .pid(
                    NarwhalIntakeOuttakeConstants.POSITION_PID_P, 
                    NarwhalIntakeOuttakeConstants.POSITION_PID_I, 
                    NarwhalIntakeOuttakeConstants.POSITION_PID_D
                )
                .maxOutput(NarwhalIntakeOuttakeConstants.PID_MAX_OUTPUT)
                .minOutput(NarwhalIntakeOuttakeConstants.PID_MIN_OUTPUT);
        
        // Creating intakeOuttakeMotor and applying configs
        intakeOuttakeMotor = new SparkMax(NarwhalIntakeOuttakeConstants.INTAKE_OUTTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        intakeOuttakeMotor.configure(intakeOuttakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      
        intakeOuttakeMotorPIDController = intakeOuttakeMotor.getClosedLoopController();
        currentState = NarwhalIntakeOuttakeState.STOPPED;
    }

    /**
     * Function to run the intake-outtake moter at a percentage of its maximum voltage.
     * 
     * @param percent percentage between -1.0 and 1.0 (negative values reverse direction)
     */
    public void setIntakeOuttakeMotorPercent(double percent){
        intakeOuttakeMotor.set(percent); // Runs using percent output of duty cycle
        currentState = NarwhalIntakeOuttakeState.CUSTOM;
    }

    /**
     * Set the intake-outtake motor to the intake motor percentage (defined in constants).
     */
    public void intake() {
        setIntakeOuttakeMotorPercent(NarwhalIntakeOuttakeConstants.INTAKE_MOTOR_PERCENT);
        currentState = NarwhalIntakeOuttakeState.INTAKING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the intake-outtake motor to the outtake motor percentage (defined in constants).
     */
    public void outtake() {
        setIntakeOuttakeMotorPercent(NarwhalIntakeOuttakeConstants.OUTTAKE_MOTOR_PERCENT);
        currentState = NarwhalIntakeOuttakeState.OUTTAKING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the intake-outtake motor to 0 percentage (will use the idle-mode to decide if it should brake or coast).
     */
    public void stop() {
        setIntakeOuttakeMotorPercent(0);
        currentState = NarwhalIntakeOuttakeState.STOPPED; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Uses a PID to actively hold the current position of the motor. (Stops the motor first).
     */
    public void hold() {
        stop();
        double current_position = intakeOuttakeMotor.getEncoder().getPosition(); // get its current position
        intakeOuttakeMotorPIDController.setReference(current_position, ControlType.kPosition); // set its current position as a PID target value so the motor holds its position
        currentState = NarwhalIntakeOuttakeState.ACTIVE_HOLDING; // must be after the stop function because the set function will default to CUSTOM state
    }

    /**
     * Will return true when pressed, and false when not pressed
     * @return boolean
     */
    public boolean isCoralSensorTriggered(){
        // return !intakeSensor.get(); // negated so that true means it is pressed, and false means it is not
        return false; // disabled
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // // This method will be called once per scheduler run during simulation
    }
}

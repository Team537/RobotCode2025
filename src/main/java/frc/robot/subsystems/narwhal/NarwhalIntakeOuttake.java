// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.narwhal;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    
    private final SparkMax intakeOuttakeMotorSparkMax;
    private final SparkMaxConfig intakeOuttakeMotorSparkMaxConfig;
    private final SparkClosedLoopController intakeOuttakeMotorPIDController;

    private final NarwhalElevator narwhalElevatorReference;

    // private final DigitalInput limitSwitch;

    public NarwhalIntakeOuttake(NarwhalElevator narwhalElevatorReference) { 
        this.narwhalElevatorReference = narwhalElevatorReference;

        intakeOuttakeMotorSparkMaxConfig = new SparkMaxConfig();
        intakeOuttakeMotorSparkMaxConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.INTAKE_OUTTAKE_MOTOR_CURRENT_LIMIT);
        intakeOuttakeMotorSparkMaxConfig.closedLoop
                .pid(
                    Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.POSITION_PID_P, 
                    Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.POSITION_PID_I, 
                    Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.POSITION_PID_D
                )
                .maxOutput(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.PID_MAX_OUTPUT)
                .minOutput(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.PID_MIN_OUTPUT);
                // .positionWrappingEnabled(true, ClosedLoopSlot.kSlot1);
        
        intakeOuttakeMotorSparkMax = new SparkMax(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.INTAKE_OUTTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        intakeOuttakeMotorSparkMax.configure(intakeOuttakeMotorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        intakeOuttakeMotorPIDController = intakeOuttakeMotorSparkMax.getClosedLoopController();
        currentState = NarwhalIntakeOuttakeState.STOPPED;

        // limitSwitch = new DigitalInput(0);
    }

    /**
     * Function to run the intake-outtake moter at a percentage of its maximum voltage.
     * @param RPM percentage between -1.0 and 1.0 (negative values reverse direction)
     */
    public void setIntakeOuttakeMotorOffset(double offset_rotations){
        double target_rotation = offset_rotations + intakeOuttakeMotorSparkMax.getEncoder().getPosition();
        intakeOuttakeMotorPIDController.setReference(target_rotation, ControlType.kPosition); // Runs using percent output of duty cycle
        currentState = NarwhalIntakeOuttakeState.CUSTOM;
    }

    
    /**
     * Function to run the intake-outtake moter at a percentage of its maximum voltage.
     * @param RPM percentage between -1.0 and 1.0 (negative values reverse direction)
     */
    public void setIntakeOuttakeMotorPower(double power){
        intakeOuttakeMotorSparkMax.set(power); // Runs using percent output of duty cycle
        currentState = NarwhalIntakeOuttakeState.CUSTOM;
    }

    /**
     * Set the intake-outtake motor to the intake motor percentage (defined in constants).
     */
    public void intake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        setIntakeOuttakeMotorPower(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.INTAKE_MOTOR_PERCENT);

        currentState = NarwhalIntakeOuttakeState.INTAKING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the intake-outtake motor to the outtake motor percentage (defined in constants).
     */
    public void outtake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        setIntakeOuttakeMotorPower(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.OUTTAKE_MOTOR_PERCENT);
        currentState = NarwhalIntakeOuttakeState.OUTTAKING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the intake-outtake motor to 0 percentage (assumes idle-mode is breaking).
     */
    public void stop() {
        intakeOuttakeMotorSparkMax.set(0);
        currentState = NarwhalIntakeOuttakeState.STOPPED; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Uses a PID to actively hold the current position of the motor. (Stops the motor first).
     */
    public void hold() {
        stop();
        double current_position = intakeOuttakeMotorSparkMax.getEncoder().getPosition();
        intakeOuttakeMotorPIDController.setReference(current_position, ControlType.kPosition);

        currentState = NarwhalIntakeOuttakeState.ACTIVE_HOLDING; // must be after the stop function because the set function will default to CUSTOM state
    }
    
    public void runXBoxController(XboxController xboxController){
        if(xboxController.getRightBumperButton() && narwhalElevatorReference.currentState == NarwhalElevatorState.INTAKE){// && !isLimitSwitchPressed()){
            intake();
        }
        else if (xboxController.getXButton()){// && isLimitSwitchPressed()){
            outtake();
        }
        else {
            stop();
        }
    }

    // public boolean isLimitSwitchPressed(){
    //     return false; // limitSwitch.get();
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {

    }
}

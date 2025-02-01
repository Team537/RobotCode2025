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
    
    public NarwhalIntakeOuttake() {
        intakeOuttakeMotorSparkMaxConfig = new SparkMaxConfig();
        intakeOuttakeMotorSparkMaxConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.INTAKE_OUTTAKE_MOTOR_CURRENT_LIMIT);
        intakeOuttakeMotorSparkMaxConfig.closedLoop
                .pid(
                    Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.POSITION_PID_P, 
                    Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.POSITION_PID_I, 
                    Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.POSITION_PID_D
                )
                .positionWrappingEnabled(true);
        
        intakeOuttakeMotorSparkMax = new SparkMax(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.INTAKE_OUTTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        intakeOuttakeMotorSparkMax.configure(intakeOuttakeMotorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        intakeOuttakeMotorPIDController = intakeOuttakeMotorSparkMax.getClosedLoopController();
        currentState = NarwhalIntakeOuttakeState.STOPPED;
    }

    /**
     * Function to run the intake-outtake moter at a percentage of its maximum voltage.
     * @param percent percentage between -1.0 and 1.0 (negative values reverse direction)
     */
    public void setIntakeOuttakeMotorPercent(double percent){
        double output_percent = Math.min(Math.max(-1.0, percent), 1.0); // Clamps the value of percent. Also prevents from the original variable being edited.
        intakeOuttakeMotorSparkMax.set(output_percent); // Runs using percent output of duty cycle
        currentState = NarwhalIntakeOuttakeState.CUSTOM;
    }

    /**
     * Set the intake-outtake motor to the intake motor percentage (defined in constants).
     */
    public void intake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        setIntakeOuttakeMotorPercent(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.INTAKE_MOTOR_PERCENTAGE);

        currentState = NarwhalIntakeOuttakeState.INTAKING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the intake-outtake motor to the outtake motor percentage (defined in constants).
     */
    public void outtake() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        setIntakeOuttakeMotorPercent(Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants.OUTTAKE_MOTOR_PERCENTAGE);
        currentState = NarwhalIntakeOuttakeState.OUTTAKING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the intake-outtake motor to 0 percentage (assumes idle-mode is breaking).
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
        double current_position = intakeOuttakeMotorSparkMax.getEncoder().getPosition();
        intakeOuttakeMotorPIDController.setReference(current_position, ControlType.kPosition);

        currentState = NarwhalIntakeOuttakeState.ACTIVE_HOLDING; // must be after the stop function because the set function will default to CUSTOM state
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    // // for simming
    // double last_called = 0;
    // int stage = 0;
    // double init = -1.0;

    @Override
    public void simulationPeriodic() {
        // // for simulation sorry the code is terrible
        // if (init == -1.0) init = System.currentTimeMillis()/1000;

        // if(System.currentTimeMillis()/1000 - last_called > 0.2){
        //     last_called = System.currentTimeMillis()/1000;
        // }
        // else return;

        // // This method will be called once per scheduler run during simulation
        // System.out.println(CurrentState.toString());
        
        // if(System.currentTimeMillis()/1000 - init > 5.0 && stage == 0){
        //     stage++;
        //     intake();
        //     init = System.currentTimeMillis()/1000;
        // }
        // if(System.currentTimeMillis()/1000 - init > 5.0 && stage == 1){
        //     stage++;
        //     stop();
        //     init = System.currentTimeMillis()/1000;
        // }
        // if(System.currentTimeMillis()/1000 - init > 5.0 && stage == 2){
        //     stage++;
        //     outtake();
        //     init = System.currentTimeMillis()/1000;
        // }
        // if(System.currentTimeMillis()/1000 - init > 5.0 && stage == 3){
        //     stage++;
        //     hold();
        //     init = System.currentTimeMillis()/1000;
        // }
        // if(System.currentTimeMillis()/1000 - init > 5.0 && stage == 4){
        //     stage++;
        //     setIntakeOuttakeMotorPercent(1.2);
        //     init = System.currentTimeMillis()/1000;
        // }
        // if(System.currentTimeMillis()/1000 - init > 5.0 && stage == 5){
        //     stage++;
        //     setIntakeOuttakeMotorPercent(-1.2);
        //     init = System.currentTimeMillis()/1000;
        // }
    }
}

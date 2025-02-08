// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.narwhal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.NarwhalConstants.NarwhalClimberConstants;

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
    
    private final TalonFX climber;
    private final PositionVoltage climberRequest;
    
    public NarwhalClimber() {
        // a whole lotta stuff for the spark max config
        climber = new TalonFX(NarwhalClimberConstants.CLIMBER_CAN_ID, "rio");

        // Motor configs
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();
        // General Configs
        climberConfig.Feedback.SensorToMechanismRatio = NarwhalClimberConstants.GEAR_REDUCTION;
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climberConfig.MotorOutput.Inverted = NarwhalClimberConstants.INVERTED;
        // PID Configs
        climberConfig.Slot0.kS = NarwhalClimberConstants.PID_FEEDFORWARD_S;
        climberConfig.Slot0.kV = NarwhalClimberConstants.PID_FEEDFORWARD_V;
        climberConfig.Slot0.kP = NarwhalClimberConstants.PID_FEEDFORWARD_P;
        climberConfig.Slot0.kI = NarwhalClimberConstants.PID_FEEDFORWARD_I;
        climberConfig.Slot0.kD = NarwhalClimberConstants.PID_FEEDFORWARD_D;
        climber.getConfigurator().apply(climberConfig);
        
        climberRequest = new PositionVoltage(0).withSlot(0);
        currentState = NarwhalClimberState.STARTING;
    }

    /**
     * Function to move the climber piece to a target angle (relative to the world) & sets the current state to CUSTOM.
     * @param targetAngle Rotation2d target position
     */
    public void setCurrentMotorAngle(Rotation2d targetAngle){
        double targetAngleRotations = targetAngle.getRotations();
        climberRequest.Position = targetAngleRotations / NarwhalClimberConstants.GEAR_REDUCTION;
        climber.setControl(climberRequest);
        currentState = NarwhalClimberState.CUSTOM;
    }

    /**
     * Set the wrist motor to the intake angle (defined in constants) & update status.
     */
    public void goToDeploy() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        setCurrentMotorAngle(NarwhalClimberConstants.DEPLOYED_ANGLE);
        currentState = NarwhalClimberState.DEPLOYING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the wrist motor to the outtake angle (defined in constants) & update status.
     */
    public void climb() {
        setCurrentMotorAngle(NarwhalClimberConstants.CLIMB_ANGLE);
        currentState = NarwhalClimberState.CLIMBING; // must be after the set function because the set function will default to CUSTOM state
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

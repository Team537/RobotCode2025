// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.narwhal;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class NarwhalWrist extends SubsystemBase {
    public NarwhalWristState CurrentState;

    private final SparkMax wrist;
    private final SparkMaxConfig wristConfig;
    private final SparkClosedLoopController wristMotorPIDController;
    
    public NarwhalWrist() {
        // a whole lotta stuff for the spark max config
        wristConfig = new SparkMaxConfig();
        // general configs
        wristConfig 
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.NarwhalConstants.NarwhalWristConstants.WRIST_MOTOR_CURRENT_LIMIT);
        // configs for the PID
        wristConfig.closedLoop 
            .pid(
                Constants.NarwhalConstants.NarwhalWristConstants.POSITION_PID_P, 
                Constants.NarwhalConstants.NarwhalWristConstants.POSITION_PID_I, 
                Constants.NarwhalConstants.NarwhalWristConstants.POSITION_PID_D
            )
            .outputRange(-0.5, 0.5)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder); // uses external encoder
        // configs for the encoder
        // NOTE FOR THE ENCODER: WHEN VIEWED FROM THE RIGHT, THE ANGLE OF THE WRIST IS BASED ON A UNIT CIRCLE WITH 0 DEGREES POINTING STRAIGHT UP
        wristConfig.encoder
            .positionConversionFactor(Constants.NarwhalConstants.NarwhalWristConstants.MOTOR_ROTATIONS_TO_WORLD_RADIANS)
            .velocityConversionFactor(Constants.NarwhalConstants.NarwhalWristConstants.MOTOR_ROTATIONS_TO_WORLD_RADIANS/60.0); // dividing by 60 accounts for RPM to Radians/Sec
        // creating the spark max controller
        wrist = new SparkMax(Constants.NarwhalConstants.NarwhalWristConstants.WRIST_MOTOR_CAN_ID, MotorType.kBrushless);
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wristMotorPIDController = wrist.getClosedLoopController();
        CurrentState = NarwhalWristState.STOPPED;
    }

    /**
     * Function to move the motor to a target angle & sets the current state to CUSTOM.
     * @param percent percentage between -1.0 and 1.0 (negative values reverse direction)
     */
    public void setCurrentMotorAngle(Rotation2d targetAngle){
        double targetAngleRadians = targetAngle.getRadians();
        wristMotorPIDController.setReference(targetAngleRadians, ControlType.kPosition);
        CurrentState = NarwhalWristState.CUSTOM;
    }

    /**
     * Set the wrist motor to the intake angle (defined in constants) & update status.
     */
    public void goToIntakeAngle() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        setCurrentMotorAngle(Constants.NarwhalConstants.NarwhalWristConstants.INTAKE_ANGLE);
        CurrentState = NarwhalWristState.INTAKING; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the wrist motor to the outtake angle (defined in constants) & update status.
     */
    public void goToOuttakeAngle() {
        setCurrentMotorAngle(Constants.NarwhalConstants.NarwhalWristConstants.OUTTAKE_ANGLE);
        CurrentState = NarwhalWristState.OUTTAKING; // must be after the set function because the set function will default to CUSTOM state
    }
    
    /**
     * Set the wrist motor to the algae descore angle (defined in constants) & update status.
     */
    public void goToAlgaeAngle() {
        setCurrentMotorAngle(Constants.NarwhalConstants.NarwhalWristConstants.ALGAE_ANGLE);
        CurrentState = NarwhalWristState.ALGAE; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Set the wrist motor to 0 percent output (assumes idle-mode is breaking).
     */
    public void stop() {
        wrist.set(0); // TODO: test that this works & properly disables the PID
        CurrentState = NarwhalWristState.STOPPED; // must be after the set function because the set function will default to CUSTOM state
    }

    /**
     * Uses a PID to actively hold the position of the motor when this function is called.
     */
    public void hold() {
        double current_position = wrist.getAbsoluteEncoder().getPosition();
        setCurrentMotorAngle(Rotation2d.fromRadians(current_position));
        CurrentState = NarwhalWristState.CUSTOM; // redundant but helps with readability
    }

    /**
     * Returns the encoder position in Rotation2d.
     */
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(wrist.getAbsoluteEncoder().getPosition());
    }

    public void runXBoxController(XboxController xBoxController){
        if(xBoxController.getRightBumperButton()){
            goToIntakeAngle();
        }
        else if(xBoxController.getLeftBumperButtonPressed()){
            goToAlgaeAngle();
        }
        else if(xBoxController.getBackButton() || xBoxController.getAButton() || xBoxController.getBButton() || xBoxController.getYButton()){
            goToOuttakeAngle();
        }
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

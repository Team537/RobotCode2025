package frc.robot.subsystems.squid;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.SquidConstants.SquidClimberConstants;
import frc.robot.util.SquidClimberPosition;

public class SquidClimber extends SubsystemBase {
    
    private SparkMax climbMotor = new SparkMax(SquidClimberConstants.CLIMB_MOTOR_CAN_ID, MotorType.kBrushless);
    private SquidClimberPosition targetClimberPosition = SquidClimberPosition.DOWN;

    public SquidClimber() {
        SparkMaxConfig climbConfig = new SparkMaxConfig();
        climbConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(SquidClimberConstants.KP, SquidClimberConstants.KI, SquidClimberConstants.KD)
                .outputRange(SquidClimberConstants.PID_MIN_OUTPUT, SquidClimberConstants.PID_MAX_OUTPUT);
        climbConfig
            .encoder
                .positionConversionFactor(SquidClimberConstants.ENCODER_FACTOR)
                .velocityConversionFactor(SquidClimberConstants.ENCODER_FACTOR / 60.0);
        climbConfig.smartCurrentLimit(SquidClimberConstants.MOTOR_CURRENT_LIMIT);
        climbConfig.idleMode(SquidClimberConstants.IDLE_MODE);
        climbConfig.inverted(SquidClimberConstants.MOTOR_INVERTED);
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Move the climber to a preset position using PID
    public void setPosition(SquidClimberPosition squidClimberPosition) {
        targetClimberPosition = squidClimberPosition;
        climbMotor.getClosedLoopController().setReference(targetClimberPosition.getPosition(), ControlType.kPosition);
    }

    // Get the current climber position (in meters)
    public double getCurrentPosition() {
        return climbMotor.getEncoder().getPosition();
    }

    /**
     * gets the squid climber position
     * @return the position of the squid climber
     */
    public SquidClimberPosition getTargetClimberPosition() {
        return targetClimberPosition;
    }

}

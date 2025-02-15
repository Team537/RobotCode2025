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

/**
 * The {@code SquidClimber} subsystem controls the climbing mechanism of the robot.
 * <p>
 * It uses a SparkMax motor controller configured for closed-loop PID control to move the climber
 * to preset positions defined by {@link SquidClimberPosition}.
 * </p>
 */
public class SquidClimber extends SubsystemBase {

    private final SparkMax climbMotor = new SparkMax(SquidClimberConstants.CLIMB_MOTOR_CAN_ID, MotorType.kBrushless);
    private SquidClimberPosition targetClimberPosition = SquidClimberPosition.DOWN;

    /**
     * Constructs a new {@code SquidClimber} and configures the SparkMax motor controller.
     */
    public SquidClimber() {
        SparkMaxConfig climbConfig = new SparkMaxConfig();
        // Configure closed-loop PID settings
        climbConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(SquidClimberConstants.KP, SquidClimberConstants.KI, SquidClimberConstants.KD)
                .outputRange(SquidClimberConstants.PID_MIN_OUTPUT, SquidClimberConstants.PID_MAX_OUTPUT);
        // Configure encoder conversion factors
        climbConfig
            .encoder
                .positionConversionFactor(SquidClimberConstants.ENCODER_FACTOR)
                .velocityConversionFactor(SquidClimberConstants.ENCODER_FACTOR / 60.0);
        // Configure current limit, idle mode, and inversion
        climbConfig.smartCurrentLimit(SquidClimberConstants.MOTOR_CURRENT_LIMIT);
        climbConfig.idleMode(SquidClimberConstants.IDLE_MODE);
        climbConfig.inverted(SquidClimberConstants.MOTOR_INVERTED);
        // Apply configuration to the motor
        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Moves the climber to the specified preset position using PID control.
     *
     * @param squidClimberPosition The target climber position.
     */
    public void setPosition(SquidClimberPosition squidClimberPosition) {
        targetClimberPosition = squidClimberPosition;
        climbMotor.getClosedLoopController().setReference(targetClimberPosition.getPosition(), ControlType.kPosition);
    }

    /**
     * Gets the current position of the climber.
     *
     * @return The climber's current position in meters.
     */
    public double getCurrentPosition() {
        return climbMotor.getEncoder().getPosition();
    }

    /**
     * Returns the target position that the climber is set to move to.
     *
     * @return The target {@link SquidClimberPosition} for the climber.
     */
    public SquidClimberPosition getTargetClimberPosition() {
        return targetClimberPosition;
    }
}

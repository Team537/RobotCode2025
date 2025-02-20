package frc.robot.subsystems.squid;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SquidConstants.SquidManipulatorConstants;

/**
 * The {@code SquidManipulator} subsystem controls the manipulator mechanism that handles game elements.
 * <p>
 * It utilizes two SparkMax motor controllers (one for the top roller and one for the bottom roller)
 * along with a sensor to detect the presence of game elements (coral).
 * </p>
 */
public class SquidManipulator extends SubsystemBase {

    /**
     * Enum representing various scoring positions for the manipulator.
     * <p>
     * TODO: Replace with a proper enum if needed.
     * </p>
     */
    public enum ScoringPosition {
        L1,
        L2,
        L3,
        L4
    }

    private ScoringPosition scoringPosition = ScoringPosition.L4;
    private boolean intaking = false;

    // Motor controllers for the manipulator wheels.
    private final SparkMax manipulatorTopMotor = new SparkMax(SquidManipulatorConstants.TOP_MOTOR_CAN_ID, MotorType.kBrushless);
    private final SparkMax manipulatorBottomMotor = new SparkMax(SquidManipulatorConstants.BOTTOM_MOTOR_CAN_ID, MotorType.kBrushless);

    // Configurations for the motors.
    private final SparkMaxConfig manipulatorTopConfig = new SparkMaxConfig();
    private final SparkMaxConfig manipulatorBottomConfig = new SparkMaxConfig();

    // Digital sensor for detecting coral.
    private final DigitalInput coralSensor = new DigitalInput(SquidManipulatorConstants.CORAL_SENSOR_ID);

    /**
     * Constructs a new {@code SquidManipulator} and configures the motor controllers.
     */
    public SquidManipulator() {
        // Configure top manipulator motor
        manipulatorTopConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(SquidManipulatorConstants.KP, SquidManipulatorConstants.KI, SquidManipulatorConstants.KD)
                .outputRange(SquidManipulatorConstants.PID_MIN_OUTPUT, SquidManipulatorConstants.PID_MAX_OUTPUT);
        manipulatorTopConfig
            .encoder
                .positionConversionFactor(SquidManipulatorConstants.ENCODER_FACTOR)
                .velocityConversionFactor(SquidManipulatorConstants.ENCODER_FACTOR / 60.0);
        manipulatorTopConfig.smartCurrentLimit(SquidManipulatorConstants.MOTOR_CURRENT_LIMIT);
        manipulatorTopConfig.idleMode(SquidManipulatorConstants.IDLE_MODE);
        manipulatorTopConfig.inverted(SquidManipulatorConstants.TOP_MOTOR_INVERTED);

        // Configure bottom manipulator motor
        manipulatorBottomConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(SquidManipulatorConstants.KP, SquidManipulatorConstants.KI, SquidManipulatorConstants.KD)
                .outputRange(SquidManipulatorConstants.PID_MIN_OUTPUT, SquidManipulatorConstants.PID_MAX_OUTPUT);
        manipulatorBottomConfig
            .encoder
                .positionConversionFactor(SquidManipulatorConstants.ENCODER_FACTOR)
                .velocityConversionFactor(SquidManipulatorConstants.ENCODER_FACTOR / 60.0);
        manipulatorBottomConfig.smartCurrentLimit(SquidManipulatorConstants.MOTOR_CURRENT_LIMIT);
        manipulatorBottomConfig.idleMode(SquidManipulatorConstants.IDLE_MODE);
        manipulatorBottomConfig.inverted(SquidManipulatorConstants.BOTTOM_MOTOR_INVERTED);

        // Apply configurations to both motors
        manipulatorTopMotor.configure(manipulatorTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        manipulatorBottomMotor.configure(manipulatorBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the velocities for both the top and bottom manipulator rollers.
     *
     * @param topVelocity    The desired velocity for the top roller in meters per second (positive value outputs game element).
     * @param bottomVelocity The desired velocity for the bottom roller in meters per second (positive value outputs game element).
     */
    public void setManipulatorVelocities(double topVelocity, double bottomVelocity) {
        manipulatorTopMotor.getClosedLoopController().setReference(topVelocity, ControlType.kVelocity);
        manipulatorBottomMotor.getClosedLoopController().setReference(bottomVelocity, ControlType.kVelocity);
    }

    /**
     * Runs the manipulator at full outtake speed.
     */
    public void outtakeFullSpeed() {
        setManipulatorVelocities(SquidManipulatorConstants.MANIPULATOR_MAX_SPEED, SquidManipulatorConstants.MANIPULATOR_MAX_SPEED);
    }

    /**
     * Reverses the manipulator at full speed.
     */
    public void reverseFullSpeed() {
        setManipulatorVelocities(-SquidManipulatorConstants.MANIPULATOR_MAX_SPEED, -SquidManipulatorConstants.MANIPULATOR_MAX_SPEED);
    }

    /**
     * Outtakes the game element (coral) at an angle determined by the given scoring position.
     *
     * @param scoringPosition The scoring position that determines the outtake angle.
     */
    public void outtakeAngled(ScoringPosition scoringPosition) {
        // Set bottom roller ratio based on scoring position.
        double bottomRollerRatio;
        switch (scoringPosition) {
            case L1:
                bottomRollerRatio = SquidManipulatorConstants.L_ONE_BOTTOM_ROLLER_RATIO;
                break;
            case L2:
                bottomRollerRatio = SquidManipulatorConstants.L_TWO_BOTTOM_ROLLER_RATIO;
                break;
            case L3:
                bottomRollerRatio = SquidManipulatorConstants.L_THREE_BOTTOM_ROLLER_RATIO;
                break;
            case L4:
                bottomRollerRatio = SquidManipulatorConstants.L_FOUR_BOTTOM_ROLLER_RATIO;
                break;
            default:
                bottomRollerRatio = 1.0;
        }

        setManipulatorVelocities(SquidManipulatorConstants.MANIPULATOR_MAX_SPEED, bottomRollerRatio * SquidManipulatorConstants.MANIPULATOR_MAX_SPEED);
    }

    /**
     * Stops the manipulator rollers to hold the game element in place.
     */
    public void holdManipulator() {
        setManipulatorVelocities(0.0, 0.0);
    }

    /**
     * Sets the scoring position for the manipulator.
     *
     * @param scoringPosition The desired scoring position.
     */
    public void setScoringPosition(ScoringPosition scoringPosition) {
        this.scoringPosition = scoringPosition;
    }

    /**
     * Sets whether the manipulator is in intaking mode.
     *
     * @param intaking {@code true} if the manipulator should intake game elements; {@code false} otherwise.
     */
    public void setIntaking(boolean intaking) {
        this.intaking = intaking;
    }

    /**
     * Gets the current scoring position of the manipulator.
     *
     * @return The current {@link ScoringPosition}.
     */
    public ScoringPosition getScoringPosition() {
        return scoringPosition;
    }

    /**
     * Checks if the coral (game element) is detected by the sensor.
     *
     * @return {@code true} if the coral is sensed; {@code false} otherwise.
     */
    public boolean senseCoral() {
        return coralSensor.get();
    }
}

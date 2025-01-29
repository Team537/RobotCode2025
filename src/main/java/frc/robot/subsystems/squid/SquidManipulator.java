package frc.robot.subsystems.squid;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SquidConstants.SquidManipulatorConstants;

public class SquidManipulator extends SubsystemBase {

    //TODO: Remove this and replace with real enum
    public enum ScoringPosition {
        L1,
        L2,
        L3,
        L4
    }

    private ScoringPosition scoringPosition = ScoringPosition.L4;
    private boolean intaking = false;

    // Defining the motors
    SparkMax manipulatorTopMotor = new SparkMax(SquidManipulatorConstants.TOP_MOTOR_CAN_ID,MotorType.kBrushless);
    SparkMax manipulatorBottomMotor = new SparkMax(SquidManipulatorConstants.BOTTOM_MOTOR_CAN_ID,MotorType.kBrushless);

    // Configs for the motors
    SparkMaxConfig manipulatorTopConfig = new SparkMaxConfig();
    SparkMaxConfig manipulatorBottomConfig = new SparkMaxConfig();

    public SquidManipulator() {
        manipulatorTopConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(SquidManipulatorConstants.KP,SquidManipulatorConstants.KI,SquidManipulatorConstants.KD)
                .outputRange(SquidManipulatorConstants.PID_MIN_OUTPUT, SquidManipulatorConstants.PID_MAX_OUTPUT);
        manipulatorTopConfig
            .encoder
                .positionConversionFactor(SquidManipulatorConstants.ENCODER_FACTOR)
                .velocityConversionFactor(SquidManipulatorConstants.ENCODER_FACTOR / 60.0);
        manipulatorTopConfig.smartCurrentLimit(SquidManipulatorConstants.MOTOR_CURRENT_LIMIT);
        manipulatorTopConfig.idleMode(SquidManipulatorConstants.IDLE_MODE);
        manipulatorTopConfig.inverted(SquidManipulatorConstants.TOP_MOTOR_INVERTED);

        manipulatorBottomConfig
            .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(SquidManipulatorConstants.KP,SquidManipulatorConstants.KI,SquidManipulatorConstants.KD)
                .outputRange(SquidManipulatorConstants.PID_MIN_OUTPUT, SquidManipulatorConstants.PID_MAX_OUTPUT);
        manipulatorBottomConfig
            .encoder
                .positionConversionFactor(SquidManipulatorConstants.ENCODER_FACTOR)
                .velocityConversionFactor(SquidManipulatorConstants.ENCODER_FACTOR / 60.0);
        manipulatorBottomConfig.smartCurrentLimit(SquidManipulatorConstants.MOTOR_CURRENT_LIMIT);
        manipulatorBottomConfig.idleMode(SquidManipulatorConstants.IDLE_MODE);
        manipulatorBottomConfig.inverted(SquidManipulatorConstants.BOTTOM_MOTOR_INVERTED);

        manipulatorTopMotor.configure(manipulatorTopConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        manipulatorBottomMotor.configure(manipulatorBottomConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    }

    /**
     * sets the velocities of the individual manipulator wheels
     * @param topVelocity the velocity of the top manipulator (positive is out) in meters/sec
     * @param bottomVelocity the velocity of the bottom manipulator (positive is out) in meters/sec
     */
    public void setManipulatorVelocities(double topVelocity, double bottomVelocity) {
        manipulatorTopMotor.getClosedLoopController().setReference(topVelocity, ControlType.kVelocity);
        manipulatorBottomMotor.getClosedLoopController().setReference(bottomVelocity, ControlType.kVelocity);
    }

    /**
     * Outtake full speed
     */
    public void outtakeFullSpeed() {
        setManipulatorVelocities(SquidManipulatorConstants.MANIPULATOR_MAX_SPEED, SquidManipulatorConstants.MANIPULATOR_MAX_SPEED);
    }

    /**
     * Reverse full speed
     */
    public void reverseFullSpeed() {
        setManipulatorVelocities(-SquidManipulatorConstants.MANIPULATOR_MAX_SPEED, -SquidManipulatorConstants.MANIPULATOR_MAX_SPEED);
    }

    /**
     * Outtakes the coral at an angle
     * @param scoringPosition The scoring position (determines the angle which will be outtaked)
     */
    public void outtakeAngled(ScoringPosition scoringPosition) {

        //set different bottom ratios depending on the height
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
     * stops the manipulator to hold the coral inside of it
     */
    public void holdManipulator() {
        setManipulatorVelocities(0.0, 0.0);
    }

    /**
     * sets the scroing position
     * @param scoringPosition the scoring position that will be set
     */
    public void setScoringPosition(ScoringPosition scoringPosition) {
        this.scoringPosition = scoringPosition;
    }
    
    /**
     * sets whether the robot is in intaking mode or not
     * @param intaking the mode (true if it is intaking)
     */
    public void setIntaking(boolean intaking) {
        this.intaking = intaking;
    }

    /**
     * manipulates using the controller
     * @param controller the controller which will be read from
     */
    public void manipulateFromXBoxController(XboxController controller) {
        if (controller.getXButton() == controller.getRightBumperButton()) {
            //if both buttons are or aren't pressed, stop the intake
            holdManipulator();
        } else {
            if (controller.getXButton()) {
                if (intaking) { 
                    outtakeFullSpeed(); //intake without a differential
                } else {
                    outtakeAngled(scoringPosition); //outtake with a differential
                }
            } else {
                reverseFullSpeed(); //reversing to unjam coral
            }
        }
    }

}

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase{
    
    SparkMax drivingSparkMax;
    SparkMax turningSparkMax;
     
    SparkClosedLoopController drivingClosedLoopController;
    SparkClosedLoopController turningClosedLoopController;

    Rotation2d moduleAngularOffset;

    /**
     * Creates a swerve module
     * @param drivingCANID The CANID for the driving motor
     * @param turningCANID The CANID for the turning motor
     * @param moduleAngularOffset The offset of the module relative to the chassis
     */
    public SwerveModule(int drivingCANID, int turningCANID, Rotation2d moduleAngularOffset) {

        //Updating the module offset
        this.moduleAngularOffset = moduleAngularOffset;

        //Creating the motor objects
        drivingSparkMax = new SparkMax(drivingCANID, MotorType.kBrushless);
        turningSparkMax = new SparkMax(turningCANID, MotorType.kBrushless);

        drivingClosedLoopController = drivingSparkMax.getClosedLoopController();
        turningClosedLoopController = turningSparkMax.getClosedLoopController();

        if (drivingSparkMax instanceof Object)
        //Creating the configuration file for thr driving motor
        SparkMaxConfig drivingConfig = new SparkMaxConfig();
        drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        drivingConfig.encoder.positionConversionFactor(DriveConstants.DRIVING_ENCODER_POSITION_FACTOR);
        drivingConfig.encoder.velocityConversionFactor(DriveConstants.DRIVING_ENCODER_VELOCITY_FACTOR);
        drivingConfig.closedLoop.pidf(DriveConstants.DRIVING_KP,DriveConstants.DRIVING_KI,DriveConstants.DRIVING_KD,DriveConstants.DRIVING_FF);
        drivingConfig.closedLoop.outputRange(DriveConstants.DRIVING_PID_MIN_OUTPUT, DriveConstants.DRIVING_PID_MAX_OUTPUT);
        drivingConfig.idleMode(DriveConstants.DRIVING_MOTOR_IDLE_MODE);
        drivingConfig.smartCurrentLimit(DriveConstants.DRIVING_MOTOR_CURRENT_LIMIT);
        drivingConfig.inverted(DriveConstants.DRIVING_ENCODER_INVERTED);

        //Creating the configuration file for the turning motor
        SparkMaxConfig turningConfig = new SparkMaxConfig();
        turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        turningConfig.absoluteEncoder.positionConversionFactor(DriveConstants.TURNING_ENCODER_POSITION_FACTOR);
        turningConfig.absoluteEncoder.velocityConversionFactor(DriveConstants.TURNING_ENCODER_VELOCITY_FACTOR);
        turningConfig.closedLoop.pid(DriveConstants.TURNING_KP,DriveConstants.TURNING_KI,DriveConstants.TURNING_KD);
        turningConfig.closedLoop.outputRange(DriveConstants.TURNING_PID_MIN_OUTPUT, DriveConstants.TURNING_PID_MAX_OUTPUT);
        turningConfig.closedLoop.positionWrappingEnabled(true);
        turningConfig.closedLoop.positionWrappingInputRange(0, DriveConstants.TURNING_FACTOR);
        turningConfig.idleMode(DriveConstants.TURNING_MOTOR_IDLE_MODE);
        turningConfig.smartCurrentLimit(DriveConstants.TURNING_MOTOR_CURRENT_LIMIT);
        turningConfig.absoluteEncoder.inverted(DriveConstants.TURNING_ENCODER_INVERTED);
        turningConfig.closedLoop.positionWrappingEnabled(true);
        turningConfig.closedLoop.positionWrappingInputRange(DriveConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT, DriveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        drivingSparkMax.getEncoder().setPosition(0);
    }

    /**
     * updates the PID controller to target a new state
     * @param state the new state to target
     */
    public void setState(SwerveModuleState state) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedDesiredState.angle = state.angle.times(1.0);

        //optimze the desired state so that the robot will never rotate more than PI/2 radians
        correctedDesiredState.optimize(getPosition().angle);

        // Don't change the orientation of the turning wheels if the speed is low
        if (Math.abs(correctedDesiredState.speedMetersPerSecond) < 1e-3) {
            correctedDesiredState.angle = getPosition().angle;
        }
        
        // Setting the references on the PID controllers
        drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        turningClosedLoopController.setReference(correctedDesiredState.angle.plus(moduleAngularOffset).getRadians(), ControlType.kPosition);
            
    }

        /**
         * gets the module's position (relative to the field)
         * @return the position of the module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drivingSparkMax.getEncoder().getPosition(),
            new Rotation2d(turningSparkMax.getAbsoluteEncoder().getPosition()).minus(moduleAngularOffset)
        );
    }

    /**
     * gets the module's state (relative to the field)
     * @return the state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            drivingSparkMax.getEncoder().getVelocity(),
            new Rotation2d(turningSparkMax.getAbsoluteEncoder().getPosition()).minus(moduleAngularOffset)
        );
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.math.RateLimiter;
import frc.robot.util.math.RateLimiter2d;
import frc.robot.util.math.Vector2d;
import frc.robot.util.swerve.DrivingMotor;
import frc.robot.util.swerve.TurningMotor;

/**
 * <h2> DriveSubsystem </h2>
 * The {@code DriveSubsystem} class is a class that focuses on controlling the robot's drivetrain. It extends SubsystemBase,
 * and supports several autonomous and teleoperated features, aiding in intelligent navigation and control. It also servers
 * as the central access point for the robot's position on the field.
 * <hr>
 * @author Parker Huibregtse
 * @since v1.1.0
 * @see {@link edu.wpi.first.wpilibj2.command.SubsystemBase}
 */
public class DriveSubsystem extends SubsystemBase {

    // SwerveModule declarations for all four modules of the drivetrain

    // Front Left Swerve Module
    private SwerveModule frontLeftModule = new SwerveModule(
            DriveConstants.FRONT_LEFT_DRIVING_MOTOR_CAN_ID, // CAN ID for front left drive motor
            DriveConstants.FRONT_LEFT_TURNING_MOTOR_CAN_ID, // CAN ID for front left turning motor
            DriveConstants.FRONT_LEFT_MODULE_ANGULAR_OFFSET // Angular offset for front left module
    );

    // Front Right Swerve Module
    private SwerveModule frontRightModule = new SwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVING_MOTOR_CAN_ID, // CAN ID for front right drive motor
            DriveConstants.FRONT_RIGHT_TURNING_MOTOR_CAN_ID, // CAN ID for front right turning motor
            DriveConstants.FRONT_RIGHT_MODULE_ANGULAR_OFFSET // Angular offset for front right module
    );

    // Rear Left Swerve Module
    private SwerveModule rearLeftModule = new SwerveModule(
            DriveConstants.REAR_LEFT_DRIVING_MOTOR_CAN_ID, // CAN ID for rear left drive motor
            DriveConstants.REAR_LEFT_TURNING_MOTOR_CAN_ID, // CAN ID for rear left turning motor
            DriveConstants.REAR_LEFT_MODULE_ANGULAR_OFFSET // Angular offset for rear left module
    );

    // Rear Right Swerve Module
    private SwerveModule rearRightModule = new SwerveModule(
            DriveConstants.REAR_RIGHT_DRIVING_MOTOR_CAN_ID, // CAN ID for rear right drive motor
            DriveConstants.REAR_RIGHT_TURNING_MOTOR_CAN_ID, // CAN ID for rear right turning motor
            DriveConstants.REAR_RIGHT_MODULE_ANGULAR_OFFSET // Angular offset for rear right module
    );

    // The PID Controllers
    private Pigeon2 gyroscope = new Pigeon2(DriveConstants.GYROSCOPE_DEVICE_ID);

    // PID Controllers used to drive the robot
    private PIDController xController = new PIDController(DriveConstants.LINEAR_KP, DriveConstants.LINEAR_KI,
            DriveConstants.LINEAR_KD);
    private PIDController yController = new PIDController(DriveConstants.LINEAR_KP, DriveConstants.LINEAR_KI,
            DriveConstants.LINEAR_KD);
    private PIDController thetaController = new PIDController(DriveConstants.ROTATIONAL_KP,
            DriveConstants.ROTATIONAL_KI, DriveConstants.ROTATIONAL_KD);

    private RateLimiter2d linearRateLimiter = new RateLimiter2d(new Vector2d(0.0, 0.0),
            DriveConstants.LINEAR_MAX_ACCELERATION, DriveConstants.MAX_DELTA_TIME_RATE_LIMIT);
    private RateLimiter rotationalRateLimiter = new RateLimiter(0.0, DriveConstants.ROTATIONAL_MAX_ACCELERATION,
            DriveConstants.MAX_DELTA_TIME_RATE_LIMIT);

    //the velocity values which the periodic will attempt to accelerate towards
    private Vector2d targetLinearVelocity = new Vector2d(0.0, 0.0);
    private double targetRotationalVelocity = 0.0;

    //the last velocity values the robot commanded the modules to reach
    private Vector2d commandedLinearVelocity = new Vector2d(0.0, 0.0);
    private double commandedRotationalVelocity = 0.0;

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.DRIVE_KINEMATICS, getGyroscopeHeading(), getSwerveModulePositions(), new Pose2d());

    public DriveSubsystem() {

        // Enabling continuos movement on the thetaController, allowing it to go around the circle
        thetaController.enableContinuousInput(-Math.PI,Math.PI);
    }

    /**
     * Drives linearly and rotationally using the following parameters, applying
     * acceleration limiting
     * 
     * @param linearVelocity     The linear velocity, in meters per second, with the
     *                           X component parallel to the longer field axis and
     *                           positive away from the blue alliance wall, and the
     *                           Y component positive to the left of the blue
     *                           alliance wall
     * @param rotationalVelocity The angular velocity, in radians per second, with
     *                           positive counter-clockwise
     */
    public void drive(Vector2d linearVelocity, double rotationalVelocity) {

        // Check for invalid linear velocity components
        if (Double.isNaN(linearVelocity.getX()) || Double.isInfinite(linearVelocity.getX())) {
            linearVelocity.setX(0.0);
        }
        if (Double.isNaN(linearVelocity.getY()) || Double.isInfinite(linearVelocity.getY())) {
            linearVelocity.setY(0.0);
        }

        // Check for invalid rotational velocity
        if (Double.isNaN(rotationalVelocity) || Double.isInfinite(rotationalVelocity)) {
            rotationalVelocity = 0.0;
        }

        // Binding the linear velocity so that it can't exceed the maximum
        Vector2d boundedLinearVelocity;
        if (linearVelocity.magnitude() > DriveConstants.LINEAR_MAX_SPEED) {
            boundedLinearVelocity = linearVelocity.normalize().scale(DriveConstants.LINEAR_MAX_SPEED);
        } else {
            boundedLinearVelocity = linearVelocity;
        }

        // Binding the angular velocity so that it can't exceed the maximum
        double boundedRotationalVelocity = Math.min(Math.max(rotationalVelocity, -DriveConstants.ROTATIONAL_MAX_SPEED),
                DriveConstants.ROTATIONAL_MAX_SPEED);

        // Adjusting the target values to set the velocity
        targetLinearVelocity = boundedLinearVelocity;
        targetRotationalVelocity = boundedRotationalVelocity;
    }


    /**
     * Returns the linear velocity value from the PID controllers, as a Vector2d.
     * 
     * @param target The target position.
     * @return The linear velocity, as a Vector2d.
     */
    public Vector2d getLinearFeedback(Translation2d target) {

        // Creating a vector using the two PID controllers
        Vector2d feedback = new Vector2d(
            xController.calculate(getRobotPose().getX(), target.getX()),
            yController.calculate(getRobotPose().getY(), target.getY())
        );

        // Setting the magnitude to the htan function of its original magnitude binds it and allows it to be smoother
        return feedback.normalize().scale(Math.tanh(feedback.magnitude()));
    }

    /**
     * Returns a rotational velocity value from the PID controller, as a double.
     * 
     * @param target The target orientation.
     * @return The value to go towards.
     */
    public double getRotationalFeedback(Rotation2d target) {
        return Math.tanh(thetaController.calculate(getRobotPose().getRotation().getRadians(), target.getRadians()));
    }

    /**
     * Ignores all rate limiters and immediately tries to stop the robot as fast as possible.
     */
    public void instantStop() {
        targetLinearVelocity = new Vector2d(0.0, 0.0);
        targetRotationalVelocity = 0.0;
        linearRateLimiter.setValue(new Vector2d(0.0, 0.0));
        rotationalRateLimiter.setValue(0.0);
    }

    /**
     * Drives linearly and rotationally using the following parameters, applying
     * acceleration limiting.
     * 
     * @param linearVelocity     The linear velocity, in meters per second, with the
     *                           X component parallel to the longer field axis and
     *                           positive away from the blue alliance wall, and the
     *                           Y component positive to the left of the blue
     *                           alliance wall
     * @param rotationalVelocity The angular velocity, in radians per second, with
     *                           positive counter-clockwise
     */
    private void setModules(Vector2d linearVelocity, double rotationalVelocity) {

        // Check for invalid linear velocity components
        if (Double.isNaN(linearVelocity.getX()) || Double.isInfinite(linearVelocity.getX())) {
            linearVelocity.setX(0.0);
        }
        if (Double.isNaN(linearVelocity.getY()) || Double.isInfinite(linearVelocity.getY())) {
            linearVelocity.setY(0.0);
        }

        // Check for invalid rotational velocity
        if (Double.isNaN(rotationalVelocity) || Double.isInfinite(rotationalVelocity)) {
            rotationalVelocity = 0.0;
        }

        // Updating the commanded velocities
        commandedLinearVelocity = linearVelocity;
        commandedRotationalVelocity = rotationalVelocity;

        // Calculating the desired module states for the robot
        SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX(),
                linearVelocity.getY(),
                rotationalVelocity,
                getRobotPose().getRotation()
            )
        );
        
        // Capping the modules' speed at the maximum linear speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.LINEAR_MAX_SPEED);

        // Setting the states of the modules
        frontLeftModule.setState(swerveModuleStates[0]);
        frontRightModule.setState(swerveModuleStates[1]);
        rearLeftModule.setState(swerveModuleStates[2]);
        rearRightModule.setState(swerveModuleStates[3]);

        //System.out.println(swerveModuleStates[1].toString());
    }

    /**
     * Returns this DriveSubsystem's gyroscope heading, as a Rotation2d. 
     * 
     * @return this DriveSubsystem's gyroscope heading, as a Rotation2d. 
     */
    public Rotation2d getGyroscopeHeading() {
        return gyroscope.getRotation2d();
    }

    /**
     * Returns this DriveSubsystem's swerver modules' states.
     * 
     * @return this DriveSubsystem's swerver modules' states.
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            rearLeftModule.getState(),
            rearRightModule.getState()
        };
    }

    /**
     * Returns this DriveSubsystem's swerve modules' positions, as an array of SwerveModulePositions.
     * 
     * @return this DriveSubsystem's swerve modules' positions, as an array of SwerveModulePositions.
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        };
    }

    /**
     * Returns this DriveSubsystems SwerveDrivePoseEstimator.
     * 
     * @return This DriveSubsystems SwerveDrivePoseEstimator.
     */
    public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator() {
        return poseEstimator;
    }

    /**
     * Returns the robot's position, as a Pose2d.
     * 
     * @return The robot's position, as a Pose2d.
     */
    public synchronized Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Sets the robot's pose.
     * 
     * @param pose The Pose2d to set the robot's position to.
     */
    public void setRobotPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    /**
     * Sets the heading of the robot.
     * 
     * @param heading The Rotation2d representing the direction that the robot will believe it is facing.
     */
    public void setRobotHeading(Rotation2d heading) {
        poseEstimator.resetRotation(heading);
    }

    /**
     * Sets the 2D position of the robot without modifying the heading.
     * 
     * @param position The position to set, as a Translation2d
     */
    public void setRobotTranslation(Translation2d translation) {
        poseEstimator.resetTranslation(translation);
    }

    /**
     * Sets the driving motor of all four swerve modules.
     * 
     * @param drivingMotor The driving motor to set.
     */
    public void setDrivingMotors(DrivingMotor drivingMotor) {
        frontLeftModule.setDrivingMotor(drivingMotor);
        frontRightModule.setDrivingMotor(drivingMotor);
        rearLeftModule.setDrivingMotor(drivingMotor);
        rearRightModule.setDrivingMotor(drivingMotor);
    }

    /**
     * Sets the turning motor of all four swerve modules.
     * 
     * @param turningMotor The turning motor to set.
     */
    public void setTurningMotors(TurningMotor turningMotor) {
        frontLeftModule.setTurningMotor(turningMotor);
        frontRightModule.setTurningMotor(turningMotor);
        rearLeftModule.setTurningMotor(turningMotor);
        rearRightModule.setTurningMotor(turningMotor);
    }

    /**
     * Returns the last commanded linear velocity (after acceleration limiting).
     * 
     * @return The last commanded linear velocity, in meters per second, as a Vector2d.
     */
    public Vector2d getCommandedLinearVelocity() {
        return commandedLinearVelocity;
    }

    /**
     * Returns the last commanded rotational velocity (after acceleration limiting).
     * 
     * @return The last commanded rotational velocity, in radians per second.
     */
    public double getCommandedRotationalVelocity() {
        return commandedRotationalVelocity;
    }

    // Runs whenever the robot is active. Even when disabled.
    // Periodic method called every loop
    @Override
    public void periodic() {

        // Update the robot pose using the module states
        poseEstimator.update(getGyroscopeHeading(), getSwerveModulePositions());

        // Accelerates the velocity towards the target
        linearRateLimiter.update(targetLinearVelocity);
        rotationalRateLimiter.update(targetRotationalVelocity);

        // Driving the robot using the accelerated values
        setModules(linearRateLimiter.getValue(), rotationalRateLimiter.getValue());
    }
}
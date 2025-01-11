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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.RateLimiter;
import frc.robot.util.RateLimiter2d;
import frc.robot.util.Vector2d;

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
     * Drives the robot, ideally from a manual controller
     * @param linearVelocity the target linear velocity of the robot, only used if useTargetTranslation is false. This is measured on a unit circle
     * @param rotationalVelocity the target linear velocity of the robot, only use if useTargetRotation is false. This is a unit value
     * @param targetTranslationOffset the offset of the targetted translation
     * @param useTargetTranslation whether or not the robot should target a translation.
     * @param targetRotationOffset the offset of the targeted orientation (could also be the absolute if absolute rotation is active) In radians
     * @param useTargetRotation whether or not the robot should target an orientation
     * @param useAbsoluteRotation whether or not the robot should target an absolute position or an offset
     * @param throttle affects the max speed of the robot. 0 results in a lower max speed and 1 results in the highest max speed
     */
    private void manualDrive(
        Vector2d linearVelocity, 
        double rotationalVelocity, 
        Translation2d targetTranslationOffset, 
        boolean useTargetTranslation, 
        Rotation2d targetRotationOffset, 
        boolean useTargetRotation, 
        boolean useAbsoluteRotation, 
        double throttle) {

        //TODO: Rework to include manual pose targetting, throttle, and orientation/position lock
        
        // Curving the inputs, enabling for easier control

        double linearSpeed = linearVelocity.magnitude();
        Vector2d curvedLinearVelocity = linearVelocity.normalize().scale(Math.pow(linearSpeed,DriveConstants.LINEAR_INPUT_CURVE_POWER));

        double rotationalSpeed = Math.abs(rotationalVelocity);
        double curvedRotationalVelocity = Math.signum(rotationalVelocity) * Math.pow (rotationalSpeed,DriveConstants.ROTATION_INPUT_CURVE_POWER);

        // Set the driving
        drive(curvedLinearVelocity.scale(DriveConstants.LINEAR_MAX_SPEED),curvedRotationalVelocity * DriveConstants.ROTATIONAL_MAX_SPEED);

    }

    /**
     * Drives manually from an XBoxController
     * @param controller The controller which will control the driving
     */
    public void driveFromXBoxController(CommandXboxController controller) {
        
        //Adding the deadband to the linear velocity, or clamping it if it is too large
        Vector2d linearVelocity = new Vector2d(controller.getLeftX(), -controller.getLeftY());
        if (linearVelocity.magnitude() < OperatorConstants.XBOX_CONTROLLER_JOYSTICK_DEADMAND_RADIUS) {
            linearVelocity = new Vector2d(0, 0);
        } else if (linearVelocity.magnitude() > 1.0) {
            linearVelocity = linearVelocity.normalize();
        }

        //Applying the deadband to the rotational velocity, or clamping it if it is too large
        double rotationalVelocity = controller.getRightY();
        if (Math.abs(rotationalVelocity) < OperatorConstants.XBOX_CONTROLLER_JOYSTICK_DEADMAND_RADIUS) {
            rotationalVelocity = 0.0;
        } else if (Math.abs(rotationalVelocity) > 1.0) {
            rotationalVelocity = Math.signum(rotationalVelocity);
        }

        // TODO: Add controls for manual pose targetting

        manualDrive(
            new Vector2d(controller.getLeftX(), -controller.getLeftY()),
            controller.getRightX(),
            new Translation2d(),
            false,
            new Rotation2d(),
            false,
            false,
            1
        );

    }

    /**
     * gets a linear velocity value from the PID controllers
     * @param target the target position
     * @return the linear velocity
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
     * gets a rotational velocity value from the PID controller
     * @param target the target orientation
     * @return the value to go towards
     */
    public double getRotationalFeedback(Rotation2d target) {
        return Math.tanh(thetaController.calculate(getRobotPose().getRotation().getRadians(),target.getRadians()));
    }

    /**
     * Ignores all rate limiters and immediately tries to stop the robot as fast as
     * possible
     */
    public void instantStop() {
        targetLinearVelocity = new Vector2d(0.0, 0.0);
        targetRotationalVelocity = 0.0;
        linearRateLimiter.setValue(new Vector2d(0.0, 0.0));
        rotationalRateLimiter.setValue(0.0);
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

        //updating the commanded velocities
        commandedLinearVelocity = linearVelocity;
        commandedRotationalVelocity = rotationalVelocity;

        //Calculating the desired module states for the robot
        SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX(),
                linearVelocity.getY(),
                rotationalVelocity,
                getRobotPose().getRotation()
            )
        );
        
        //Capping the modules' speed at the maximum linear speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.LINEAR_MAX_SPEED);

        // Setting the states of the modules
        frontLeftModule.setState(swerveModuleStates[0]);
        frontRightModule.setState(swerveModuleStates[1]);
        rearLeftModule.setState(swerveModuleStates[2]);
        rearRightModule.setState(swerveModuleStates[3]);

    }

    /**
     * Gets the gyroscope heading
     * @return The gyroscope heading
     */
    public Rotation2d getGyroscopeHeading() {
        return gyroscope.getRotation2d();
    }

    /**
     * Gets the Module States
     * @return the Module States
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
     * Gets the Module Positions
     * @return the Module Positions
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
     * Gets the pose of the robot
     * @return the robot's pose
     */
    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * sets the robot's pose
     * @param pose the pose to set
     */
    public void setRobotPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    /**
     * sets the heading of the robot
     * @param heading the heading to set
     */
    public void setRobotHeading(Rotation2d heading) {
        poseEstimator.resetRotation(heading);
    }

    /**
     * sets the translation of the robot without modifying the heading
     * @param position the position to set
     */
    public void setRobotTranslation(Translation2d translation) {
        poseEstimator.resetTranslation(translation);
    }

    // Periodic method called every loop
    @Override
    public void periodic() {

        // Accelerates the velocity towards the target
        linearRateLimiter.update(targetLinearVelocity);
        rotationalRateLimiter.update(targetRotationalVelocity);

        // Driving the robot using the accelerated values
        setModules(linearRateLimiter.getValue(),rotationalRateLimiter.getValue());

        // Update the robot pose using the module states
        poseEstimator.update(getGyroscopeHeading(), getSwerveModulePositions());

    }

}

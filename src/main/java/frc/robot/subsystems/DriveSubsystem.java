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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // The rotational offset of the driver
    private Rotation2d driverRotationalOffset = new Rotation2d(0.0);

    // Variables for the linear "locking" mechanism when not commanded to drive
    boolean xyLockActive = false;
    Translation2d xyLockTranslation = new Translation2d();

    // Variables for the rotational "locking" mechanism when not commanded to turn
    boolean thetaLockActive = false;
    Rotation2d thetaLockRotation = new Rotation2d();

    // Variables to prevent the driver from moving right after targeting a location, set false to deactive driving until stopped
    boolean linearVelocityReset = true;
    boolean rotationalVelocityReset = true;

    // Variables used for linear targetting
    boolean targetTranslationActive;
    Translation2d targetTranslationOrigin = new Translation2d();

    boolean targetRotationActive = false;
    Rotation2d targetRotationOrigin = new Rotation2d();

    // Manual drive variables
    private boolean xboxOrientationOffsetTargetActive = false;

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
     * Drives the robot using manual inputs or target-based controls. This method calculates 
     * the final linear and rotational velocities based on the input parameters and applies 
     * them to the robot.
     *
     * @param linearVelocity the target linear velocity of the robot. Used when useTargetTranslation is false.
     *                       This is represented as a vector on a unit circle.
     * @param rotationalVelocity the target rotational velocity of the robot. Used when useTargetRotation is false.
     *                           This is a unit value (range [-1, 1]).
     * @param targetTranslationOffset the desired translation offset for targeting a specific position.
     * @param useTargetTranslation whether or not the robot should aim for a target translation.
     * @param targetRotationOffset the desired rotation offset for targeting a specific orientation 
     *                              (or the absolute orientation if useAbsoluteRotation is true). Measured in radians.
     * @param useTargetRotation whether or not the robot should aim for a target rotation.
     * @param useAbsoluteRotation whether the robot should target an absolute orientation or a relative offset.
     * @param throttle a multiplier for controlling the max speed of the robot. Ranges from 0 (minimum speed) to 1 (maximum speed).
     */
    private void manualDrive(
        Vector2d linearVelocity, 
        double rotationalVelocity, 
        Translation2d targetTranslationOffset, 
        boolean useTargetTranslation, 
        Rotation2d targetRotationOffset, 
        boolean useTargetRotation, 
        boolean useAbsoluteRotation, 
        double throttle
    ) {
        // The final velocities that will be used for driving the robot
        Vector2d finalLinearVelocity;
        double finalRotationalVelocity;

        // --- Handling linear velocity ---
        if (!useTargetTranslation) {
            // Target translation is not active, so use joystick input for linear control
            targetTranslationActive = false;

            // Rotate and curve the input for smoother control
            linearVelocity = linearVelocity.rotateBy(driverRotationalOffset.times(-1.0)); // Adjust for driver orientation
            double linearSpeed = linearVelocity.magnitude();
            linearVelocity = linearVelocity.normalize().scale(Math.pow(linearSpeed, DriveConstants.LINEAR_INPUT_CURVE_POWER));

            // Resetting linear velocity when no input is provided
            if (linearVelocity.magnitude() < 1e-3) {
                linearVelocityReset = true;
            }
            if (!linearVelocityReset) {
                linearVelocity = new Vector2d(0, 0);
            }

            // Lock the robot's position if no movement is commanded
            // TODO: test and fix this, delete the "false && " to activate
            if (false && linearVelocity.magnitude() < 1e-3 && commandedLinearVelocity.magnitude() < 1e-3) {
                if (!xyLockActive) {
                    xyLockActive = true;
                    xyLockTranslation = getRobotPose().getTranslation();
                }
                xyLockActive = true;
                finalLinearVelocity = getLinearFeedback(xyLockTranslation).scale(DriveConstants.LINEAR_MAX_SPEED);
            } else {
                // Apply throttle for speed control
                double linearThrottleMultiplier = OperatorConstants.THROTTLE_LINEAR_MIN_SPEED + throttle * 
                    (OperatorConstants.THROTTLE_LINEAR_MAX_SPEED - OperatorConstants.THROTTLE_LINEAR_MIN_SPEED);
                finalLinearVelocity = linearVelocity.scale(linearThrottleMultiplier);
            }
        } else {
            // Target translation is active
            targetTranslationOffset.rotateBy(driverRotationalOffset.times(-1.0)); // Adjust for driver orientation
            if (!targetTranslationActive) {
                // Initialize the origin for target translation
                targetTranslationActive = true;
                targetTranslationOrigin = getRobotPose().getTranslation().minus(targetTranslationOffset);
            }
            // Use PID feedback to calculate the velocity toward the target position
            finalLinearVelocity = getLinearFeedback(targetTranslationOrigin.plus(targetTranslationOffset)).scale(DriveConstants.LINEAR_MAX_SPEED);
            linearVelocityReset = false; // Prevent accidental movement when deactivating
        }

        // --- Handling rotational velocity ---
        if (!useTargetRotation) {
            // Target rotation is not active, so use joystick input for rotation control
            targetRotationActive = false;

            // Curve the rotational input for smoother control
            double rotationalSpeed = Math.abs(rotationalVelocity);
            rotationalVelocity = Math.signum(rotationalVelocity) * Math.pow(rotationalSpeed, DriveConstants.ROTATION_INPUT_CURVE_POWER);

            // Resetting rotational velocity when no input is provided
            if (Math.abs(rotationalVelocity) < 1e-3) {
                rotationalVelocityReset = true;
            }
            if (!rotationalVelocityReset) {
                rotationalVelocity = 0.0;
            }

            // Lock the robot's orientation if no rotation is commanded

            // TODO: test and fix this, delete the "false && " to activate
            if (false && Math.abs(rotationalVelocity) < 1e-3 && Math.abs(commandedRotationalVelocity) < 1e-3) {
                if (!thetaLockActive) {
                    thetaLockActive = true;
                    thetaLockRotation = getRobotPose().getRotation();
                }
                thetaLockActive = true;
                finalRotationalVelocity = getRotationalFeedback(thetaLockRotation) * DriveConstants.ROTATIONAL_MAX_SPEED;
            } else {
                // Apply throttle for rotational speed control
                double rotationalThrottleMultiplier = OperatorConstants.THROTTLE_ROTATIONAL_MIN_SPEED + throttle * 
                    (OperatorConstants.THROTTLE_ROTATIONAL_MAX_SPEED - OperatorConstants.THROTTLE_ROTATIONAL_MIN_SPEED);
                finalRotationalVelocity = rotationalVelocity * rotationalThrottleMultiplier;
            }
        } else {
            // Target rotation is active
            targetRotationOffset.rotateBy(driverRotationalOffset.times(-1.0)); // Adjust for driver orientation

            if (!useAbsoluteRotation) {
                // Targeting a relative rotation
                if (!targetRotationActive) {
                    // Initialize the origin for target rotation
                    targetRotationActive = true;
                    targetRotationOrigin = getRobotPose().getRotation().minus(targetRotationOffset);
                }
                // Use PID feedback to calculate the rotational velocity
                finalRotationalVelocity = getRotationalFeedback(targetRotationOrigin.rotateBy(targetRotationOffset)) * DriveConstants.ROTATIONAL_MAX_SPEED;
            } else {
                // Targeting an absolute rotation
                targetRotationActive = false; // Disable relative rotation targeting
                finalRotationalVelocity = getRotationalFeedback(targetRotationOffset) * DriveConstants.ROTATIONAL_MAX_SPEED;
            }

            rotationalVelocityReset = false; // Prevent accidental movement when deactivating
        }

        // --- Apply the calculated velocities to the robot ---
        drive(finalLinearVelocity, finalRotationalVelocity);
    }

    /**
     * Drives the robot manually using an XboxController.
     * This method handles input for linear velocity, rotational velocity, 
     * manual translation/rotation targeting, and throttle control.
     *
     * @param controller The XboxController used to control the robot.
     */
    public void driveFromXBoxController(XboxController controller) {

        // --- Linear Velocity Control ---
        // Calculate linear velocity based on the left stick input (x for strafe, y for forward/backward).
        // Apply a deadband to ignore small joystick movements and normalize the vector if it's too large.
        Vector2d linearVelocity = new Vector2d(controller.getLeftX(), -controller.getLeftY());
        if (linearVelocity.magnitude() < OperatorConstants.XBOX_CONTROLLER_JOYSTICK_DEADMAND_RADIUS) {
            linearVelocity = new Vector2d(0, 0); // Ignore small joystick movements
        } else if (linearVelocity.magnitude() > 1.0) {
            linearVelocity = linearVelocity.normalize(); // Normalize to keep within [-1, 1]
        }

        // --- Rotational Velocity Control ---
        // Calculate rotational velocity from the right stick's X-axis input.
        // Apply a deadband to ignore small movements and clamp the value to [-1, 1].
        double rotationalVelocity = -controller.getRightX();
        if (Math.abs(rotationalVelocity) < OperatorConstants.XBOX_CONTROLLER_JOYSTICK_DEADMAND_RADIUS) {
            rotationalVelocity = 0.0; // Ignore small joystick movements
        } else if (Math.abs(rotationalVelocity) > 1.0) {
            rotationalVelocity = Math.signum(rotationalVelocity); // Clamp to [-1, 1]
        }

        // --- Manual Translation Targeting ---
        // Calculate a target offset for translation based on the left stick and right trigger.
        // The trigger adjusts the radius of the target offset, ranging from minimum to maximum radius.
        // Activates when the left stick button is pressed.
        double linearTargetRadius = OperatorConstants.XBOX_CONTROLLER_TARGET_MIN_RADIUS 
            + controller.getRightTriggerAxis() * (OperatorConstants.XBOX_CONTROLLER_TARGET_MAX_RADIUS 
            - OperatorConstants.XBOX_CONTROLLER_TARGET_MIN_RADIUS);
        Translation2d targetTranslationOffset = new Translation2d(
            controller.getLeftX() * linearTargetRadius, 
            -controller.getLeftY() * linearTargetRadius
        );
        boolean useTargetTranslation = controller.getLeftStickButton();

        // --- Manual Rotation Targeting ---
        // Calculate a target offset for rotation based on the right stick vector.
        // Activates rotation targeting if:
        // - The right stick button is pressed, OR
        // - The right stick's Y-axis exceeds a specific activation zone, OR
        // - Rotation targeting was already active and the right stick magnitude is above the deactivation threshold.
        Vector2d rightStickVector = new Vector2d(controller.getRightX(), -controller.getRightY());
        Rotation2d targetRotationOffset = rightStickVector.angle();
        if (
            controller.getRightStickButton() ||
            Math.abs(controller.getRightY()) > OperatorConstants.XBOX_CONTROLLER_ROTATIONAL_TARGET_ACTIVATION_ZONE || // Activate if pushed enough
            (xboxOrientationOffsetTargetActive && rightStickVector.magnitude() > OperatorConstants.XBOX_CONTROLLER_ROTATIONAL_TARGET_DEACTIVATION_ZONE) // Stay active if above threshold
        ) {
            xboxOrientationOffsetTargetActive = true; // Enable rotation targeting
        } else {
            xboxOrientationOffsetTargetActive = false; // Disable rotation targeting
        }
        boolean useTargetRotation = xboxOrientationOffsetTargetActive;

        // Use absolute rotation if the right stick button is pressed.
        boolean useAbsoluteRotation = controller.getRightStickButton();

        // --- Throttle Control ---
        // Throttle determines the overall speed of the robot, based on the right trigger's position (0 to 1).
        double throttle = controller.getRightTriggerAxis();

        // --- Call the Manual Drive Method ---
        // Pass all calculated values to the manualDrive method for execution.
        manualDrive(
            linearVelocity,
            rotationalVelocity,
            targetTranslationOffset,
            useTargetTranslation,
            targetRotationOffset,
            useTargetRotation,
            useAbsoluteRotation,
            throttle
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
        return Math.tanh(thetaController.calculate(getRobotPose().getRotation().getRadians(), target.getRadians()));
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

        System.out.println(swerveModuleStates[1].toString());
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

    // Runs whenever the robot is active. Even when dissabled.
    // Periodic method called every loop
    @Override
    public void periodic() {

        // Update the robot pose using the module states
        poseEstimator.update(getGyroscopeHeading(), getSwerveModulePositions());
        
        // Accelerates the velocity towards the target
        linearRateLimiter.update(targetLinearVelocity);
        rotationalRateLimiter.update(targetRotationalVelocity);

        linearRateLimiter.setValue(targetLinearVelocity);
        rotationalRateLimiter.setValue(targetRotationalVelocity);

        // Driving the robot using the accelerated values
        setModules(linearRateLimiter.getValue(), rotationalRateLimiter.getValue());
    }
}

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Defaults;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.NarwhalConstants;
import frc.robot.Constants.SquidConstants;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.util.DeltaTime;
import frc.robot.util.DrivingMotorType;
import frc.robot.util.Obstacle;
import frc.robot.util.TurningMotorType;
import frc.robot.util.UpperAssemblyType;
import frc.robot.util.Vector2d;

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

    //////////////////////////////////////////////////////////////////////////////
    // Swerve Modules
    //////////////////////////////////////////////////////////////////////////////

    // Front Left Swerve Module
    private SwerveModule frontLeftModule = new SwerveModule(
            DriveConstants.FRONT_LEFT_DRIVING_MOTOR_CAN_ID,
            DriveConstants.FRONT_LEFT_TURNING_MOTOR_CAN_ID,
            DriveConstants.FRONT_LEFT_MODULE_ANGULAR_OFFSET
    );

    // Front Right Swerve Module
    private SwerveModule frontRightModule = new SwerveModule(
            DriveConstants.FRONT_RIGHT_DRIVING_MOTOR_CAN_ID,
            DriveConstants.FRONT_RIGHT_TURNING_MOTOR_CAN_ID,
            DriveConstants.FRONT_RIGHT_MODULE_ANGULAR_OFFSET
    );

    // Rear Left Swerve Module
    private SwerveModule rearLeftModule = new SwerveModule(
            DriveConstants.REAR_LEFT_DRIVING_MOTOR_CAN_ID,
            DriveConstants.REAR_LEFT_TURNING_MOTOR_CAN_ID,
            DriveConstants.REAR_LEFT_MODULE_ANGULAR_OFFSET
    );

    // Rear Right Swerve Module
    private SwerveModule rearRightModule = new SwerveModule(
            DriveConstants.REAR_RIGHT_DRIVING_MOTOR_CAN_ID,
            DriveConstants.REAR_RIGHT_TURNING_MOTOR_CAN_ID,
            DriveConstants.REAR_RIGHT_MODULE_ANGULAR_OFFSET
    );

    //////////////////////////////////////////////////////////////////////////////
    // Sensors and Controllers
    //////////////////////////////////////////////////////////////////////////////

    /** Gyroscope sensor for obtaining the robot's heading. */
    private Pigeon2 gyroscope = new Pigeon2(DriveConstants.GYROSCOPE_DEVICE_ID);

    /** PID controllers for linear (X and Y) and rotational (theta) control. */
    private PIDController xController = new PIDController(DriveConstants.LINEAR_KP, DriveConstants.LINEAR_KI, DriveConstants.LINEAR_KD);
    private PIDController yController = new PIDController(DriveConstants.LINEAR_KP, DriveConstants.LINEAR_KI, DriveConstants.LINEAR_KD);
    private PIDController thetaController = new PIDController(DriveConstants.ROTATIONAL_KP, DriveConstants.ROTATIONAL_KI, DriveConstants.ROTATIONAL_KD);

    //////////////////////////////////////////////////////////////////////////////
    // Motion and Pose Estimation
    //////////////////////////////////////////////////////////////////////////////

    /** The desired chassis speeds toward which the system will accelerate. */
    private ChassisSpeeds targetVelocities = new ChassisSpeeds();

    /** The last chassis speeds actually commanded to the swerve modules. */
    private ChassisSpeeds commandedVelocities = new ChassisSpeeds();

    /**
     * Pose estimator for the robot’s position. Note: Although this is initialized via field
     * declarations, it relies on sensor and module values that have been instantiated above.
     */
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            getGyroscopeHeading(),
            getSwerveModulePositions(),
            new Pose2d()
    );

    //////////////////////////////////////////////////////////////////////////////
    // Swerve Setpoint Generation
    //////////////////////////////////////////////////////////////////////////////

    /** Generator for swerve drive setpoints based on the current configuration. */
    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint setpoint = new SwerveSetpoint(commandedVelocities, getSwerveModuleStates(), null);
    private DeltaTime setpointDeltaTime = new DeltaTime();

    //////////////////////////////////////////////////////////////////////////////
    // Motor and Assembly Configuration
    //////////////////////////////////////////////////////////////////////////////

    private DrivingMotorType drivingMotorType = Defaults.DEFAULT_DRIVING_MOTOR;
    private TurningMotorType turningMotorType = Defaults.DEFAULT_TURNING_MOTOR;
    private UpperAssemblyType upperAssemblyType = Defaults.DEFAULT_UPPER_ASSEMBLY;

    //////////////////////////////////////////////////////////////////////////////
    // Pathfinding Configuration
    //////////////////////////////////////////////////////////////////////////////

    private PathConstraints constraints;
    private List<Supplier<List<Obstacle>>> pathfindingObstaclesSuppliers = new ArrayList<>();
    private List<Obstacle> pathfindingObstacles = new ArrayList<>();
    private List<Pair<Translation2d, Translation2d>> translatedPathfindingObstacles = new ArrayList<>();

    //////////////////////////////////////////////////////////////////////////////
    // Constructor
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Constructs a new DriveSubsystem.
     * <p>
     * This constructor initializes the PID controllers (with continuous input for the theta controller),
     * updates the drive configurations, and sets up mutable lists for dynamic pathfinding obstacles.
     * </p>
     */
    public DriveSubsystem() {
        // Enable continuous input for the rotational controller (wraps around at ±π).
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Update all configuration settings (motor types, module configurations, etc.).
        setConfigs();
    }

    //////////////////////////////////////////////////////////////////////////////
    // Configuration Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Updates the swerve drive configurations including upper assembly, motor, module, and auto-builder settings.
     */
    public void setConfigs() {
        // --- Upper Assembly Configuration ---
        double upperAssemblyMass;
        double upperAssemblyMOI;
        switch (upperAssemblyType) {
            case SQUID:
                upperAssemblyMass = SquidConstants.UPPER_ASSEMBLY_MASS;
                upperAssemblyMOI = SquidConstants.UPPER_ASSEMBLY_MOI;
                break;
            case NARWHAL:
                upperAssemblyMass = NarwhalConstants.UPPER_ASSEMBLY_MASS;
                upperAssemblyMOI = NarwhalConstants.UPPER_ASSEMBLY_MOI;
                break;
            case NONE:
            default:
                upperAssemblyMass = 0.0;
                upperAssemblyMOI = 0.0;
                break;
        }

        // Combine drivetrain and upper assembly properties.
        double robotMass = DriveConstants.DRIVETRAIN_MASS + upperAssemblyMass;
        double robotMOI = DriveConstants.DRIVETRAIN_MOI + upperAssemblyMOI;

        // --- Driving Motor Configuration ---
        double maxDriveVelocity;
        DCMotor motorType;
        double motorReduction;
        int currentLimit;
        switch (drivingMotorType) {
            case NEO:
                maxDriveVelocity = DriveConstants.NeoDriving.DRIVE_WHEEL_FREE_SPEED;
                motorType = DCMotor.getNEO(1);
                motorReduction = DriveConstants.NeoDriving.MOTOR_REDUCTION;
                currentLimit = DriveConstants.NeoDriving.CURRENT_LIMIT;
                break;
            case KRAKEN_X60:
                maxDriveVelocity = DriveConstants.KrakenX60Driving.DRIVE_WHEEL_FREE_SPEED;
                motorType = DCMotor.getKrakenX60(1);
                motorReduction = DriveConstants.KrakenX60Driving.MOTOR_REDUCTION;
                currentLimit = DriveConstants.KrakenX60Driving.CURRENT_LIMIT;
                break;
            case KRAKEN_X60_FOC:
                maxDriveVelocity = DriveConstants.KrakenX60FOCDriving.DRIVE_WHEEL_FREE_SPEED;
                motorType = DCMotor.getKrakenX60Foc(1);
                motorReduction = DriveConstants.KrakenX60FOCDriving.MOTOR_REDUCTION;
                currentLimit = DriveConstants.KrakenX60FOCDriving.CURRENT_LIMIT;
                break;
            default:
                maxDriveVelocity = 0.0;
                motorType = null;
                motorReduction = 1.0;
                currentLimit = 0;
                break;
        }

        // --- Module Configuration ---
        ModuleConfig moduleConfig = new ModuleConfig(
                DriveConstants.WHEEL_RADIUS,
                maxDriveVelocity,
                DriveConstants.WHEEL_COEFFICIENT_FRICTION,
                motorType,
                motorReduction,
                currentLimit,
                4  // Number of swerve modules
        );

        // --- Turning Motor Configuration ---
        double maxTurningSpeed;
        switch (turningMotorType) {
            case NEO_550:
                maxTurningSpeed = DriveConstants.Neo550Turning.MAX_TURNING_SPEED;
                break;
            default:
                maxTurningSpeed = 0.0;
                break;
        }

        // --- Controller Configuration ---
        PPHolonomicDriveController controller = new PPHolonomicDriveController(
                new PIDConstants(DriveConstants.LINEAR_KP, DriveConstants.LINEAR_KI, DriveConstants.LINEAR_KD),
                new PIDConstants(DriveConstants.ROTATIONAL_KP, DriveConstants.ROTATIONAL_KI, DriveConstants.ROTATIONAL_KD)
        );

        // Calculate maximum translational acceleration using friction and gravity.
        double maxTranslationalAcceleration = DriveConstants.WHEEL_COEFFICIENT_FRICTION * DriveConstants.GRAVITY_ACCELERATION;

        // Determine the effective radius of the drivetrain.
        double effectiveRadius = Math.hypot(DriveConstants.WHEEL_BASE / 2.0, DriveConstants.TRACK_WIDTH / 2.0);

        // Compute maximum angular velocity and acceleration.
        double maxAngularVelocity = maxDriveVelocity / effectiveRadius;
        double maxAngularAcceleration = (DriveConstants.WHEEL_COEFFICIENT_FRICTION * DriveConstants.GRAVITY_ACCELERATION * robotMass * effectiveRadius) / robotMOI;

        // --- Swerve and Auto-Builder Configuration ---
        RobotConfig swerveConfig = new RobotConfig(robotMass, robotMOI, moduleConfig, DriveConstants.MODULE_POSITIONS);
        constraints = new PathConstraints(maxDriveVelocity, maxTranslationalAcceleration, maxAngularVelocity, maxAngularAcceleration);

        AutoBuilder.configure(
                this::getRobotPose,                  // Supplier for current pose
                this::setRobotPose,                  // Consumer to update pose
                this::getChassisSpeeds,              // Supplier for chassis speeds
                (chassisSpeeds, feedforward) -> driveRobotRelative(chassisSpeeds), // Drive command (field-relative)
                controller,                          // Holonomic drive controller
                swerveConfig,                        // Swerve configuration
                () -> false,                         // (Optional) condition for additional behavior
                this
        );

        // --- Setpoint Generator Initialization ---
        setpointGenerator = new SwerveSetpointGenerator(swerveConfig, maxTurningSpeed);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Driving Command Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Commands the robot to drive in a field-relative manner.
     *
     * @param velocities the desired chassis speeds (m/s and rad/s)
     */
    public void drive(ChassisSpeeds velocities) {
        targetVelocities = velocities;
    }

    /**
     * Commands the robot to drive using robot-relative speeds.
     *
     * @param velocities the desired chassis speeds (m/s and rad/s)
     */
    public void driveRobotRelative(ChassisSpeeds velocities) {
        targetVelocities = ChassisSpeeds.fromRobotRelativeSpeeds(velocities, getRobotPose().getRotation());
    }

    /**
     * Updates the states of the swerve modules based on the current target velocities.
     *
     * @param targetVelocities the desired chassis speeds (m/s and rad/s)
     */
    private void setModules(ChassisSpeeds targetVelocities) {
        // Generate a new setpoint from the previous one.
        setpoint = setpointGenerator.generateSetpoint(setpoint, targetVelocities, setpointDeltaTime.getDeltaTime());

        // Update the record of commanded velocities.
        commandedVelocities = ChassisSpeeds.fromRobotRelativeSpeeds(
                DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(setpoint.moduleStates()),
                getRobotPose().getRotation()
        );

        // Command each swerve module with its new state.
        frontLeftModule.setState(setpoint.moduleStates()[0]);
        frontRightModule.setState(setpoint.moduleStates()[1]);
        rearLeftModule.setState(setpoint.moduleStates()[2]);
        rearRightModule.setState(setpoint.moduleStates()[3]);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Feedback and Navigation Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Computes linear feedback for the drive system based on a target position.
     *
     * @param target the target translation (meters)
     * @return a normalized vector (scaled by tanh of its magnitude) representing the feedback
     */
    public Vector2d getLinearFeedback(Translation2d target) {
        Vector2d feedback = new Vector2d(
                xController.calculate(getRobotPose().getX(), target.getX()),
                yController.calculate(getRobotPose().getY(), target.getY())
        );
        return feedback.normalize().scale(Math.tanh(feedback.magnitude()));
    }

    /**
     * Computes rotational feedback for the drive system based on a target orientation.
     *
     * @param target the target rotation
     * @return the rotational feedback value (scaled via tanh)
     */
    public double getRotationalFeedback(Rotation2d target) {
        return Math.tanh(thetaController.calculate(getRobotPose().getRotation().getRadians(), target.getRadians()));
    }

    /**
     * Generates a command that drives the robot to a specific pose
     * 
     * @param pose the target Pose2d to reach
     * @return a Command that, when executed, will drive to the target pose
     */
    public Command getDriveToPoseCommand(Pose2d pose) {
        return new DriveToPoseCommand(this, pose, DriveConstants.TRANSLATION_THRESHOLD,DriveConstants.ROTATION_THRESHOLD);
    }

    /**
     * Generates a command that uses pathfinding to drive the robot to a specified pose.
     *
     * @param pose the target Pose2d to reach
     * @return a Command that, when executed, will pathfind to the target pose
     */
    public Command getPathfindingCommand(Pose2d pose) {
        return new SequentialCommandGroup(
            AutoBuilder.pathfindToPose(pose, constraints, 0.0),
            getDriveToPoseCommand(pose)
        );
    }

    //////////////////////////////////////////////////////////////////////////////
    // Sensor and Pose Estimation Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Retrieves the current heading from the gyroscope.
     *
     * @return the robot's current rotation as reported by the gyroscope
     */
    public Rotation2d getGyroscopeHeading() {
        return gyroscope.getRotation2d();
    }

    /**
     * Returns the current states of all swerve modules.
     *
     * @return an array of SwerveModuleState representing each module's state
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
     * Returns the current positions of all swerve modules.
     *
     * @return an array of SwerveModulePosition representing each module's position
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
     * Retrieves the current chassis speeds computed from the module states.
     *
     * @return the chassis speeds (m/s and rad/s)
     */
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    }

    /**
     * Returns the current estimated pose of the robot.
     *
     * @return the robot's Pose2d (position and orientation)
     */
    public synchronized Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the robot's pose estimator to a specified pose.
     *
     * @param pose the new Pose2d to set for the robot
     */
    public void setRobotPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    /**
     * Resets only the robot's heading in the pose estimator.
     *
     * @param heading the new heading (Rotation2d)
     */
    public void setRobotHeading(Rotation2d heading) {
        poseEstimator.resetRotation(heading);
    }

    /**
     * Resets only the robot's translation (position) in the pose estimator.
     *
     * @param translation the new position (Translation2d)
     */
    public void setRobotTranslation(Translation2d translation) {
        poseEstimator.resetTranslation(translation);
    }

    /**
     * Returns the SwerveDrivePoseEstimator used by the subsystem.
     *
     * @return the SwerveDrivePoseEstimator instance
     */
    public SwerveDrivePoseEstimator getSwerveDrivePoseEstimator() {
        return poseEstimator;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Motor and Assembly Configuration Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Sets the driving motor type for all swerve modules.
     *
     * @param drivingMotorType the desired driving motor type
     */
    public void setDrivingMotors(DrivingMotorType drivingMotorType) {
        frontLeftModule.setDrivingMotor(drivingMotorType);
        frontRightModule.setDrivingMotor(drivingMotorType);
        rearLeftModule.setDrivingMotor(drivingMotorType);
        rearRightModule.setDrivingMotor(drivingMotorType);
        this.drivingMotorType = drivingMotorType;
    }

    /**
     * Sets the turning motor type for all swerve modules.
     *
     * @param turningMotorType the desired turning motor type
     */
    public void setTurningMotors(TurningMotorType turningMotorType) {
        frontLeftModule.setTurningMotor(turningMotorType);
        frontRightModule.setTurningMotor(turningMotorType);
        rearLeftModule.setTurningMotor(turningMotorType);
        rearRightModule.setTurningMotor(turningMotorType);
        this.turningMotorType = turningMotorType;
    }

    /**
     * Sets the upper assembly type, affecting overall robot dynamics.
     *
     * @param upperAssemblyType the desired upper assembly type
     */
    public void setUpperAssembly(UpperAssemblyType upperAssemblyType) {
        this.upperAssemblyType = upperAssemblyType;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Pathfinding Obstacle Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Adds a supplier that provides pathfinding obstacles.
     *
     * @param pathfindingObstaclesSupplier a supplier returning a list of obstacles
     */
    public void addPathfindingObstaclesSupplier(Supplier<List<Obstacle>> pathfindingObstaclesSupplier) {
        this.pathfindingObstaclesSuppliers.add(pathfindingObstaclesSupplier);
    }

    /**
     * Removes a supplier of pathfinding obstacles.
     *
     * @param pathfindingObstaclesSupplier the supplier to remove
     */
    public void removePathfindingObstaclesSupplier(Supplier<List<Obstacle>> pathfindingObstaclesSupplier) {
        this.pathfindingObstaclesSuppliers.remove(pathfindingObstaclesSupplier);
    }

    /**
     * Translates circular obstacles into their inscribed square corners for pathfinding.
     *
     * @param obstacles a list of circular obstacles
     * @return a list of pairs representing the bottom-left and top-right corners of the inscribed squares
     */
    private List<Pair<Translation2d, Translation2d>> translatePathfindingObstacles(List<Obstacle> obstacles) {
        List<Pair<Translation2d, Translation2d>> inscribedSquareCorners = new ArrayList<>();
        for (Obstacle obstacle : obstacles) {
            Translation2d center = obstacle.getObstacleTranslation();
            double radius = obstacle.getObstacleRadius();
            // Calculate the inscribed square's bottom-left and top-right corners.
            Translation2d bottomLeft = new Translation2d(center.getX() - radius, center.getY() - radius);
            Translation2d topRight = new Translation2d(center.getX() + radius, center.getY() + radius);
            inscribedSquareCorners.add(new Pair<>(bottomLeft, topRight));
        }
        return inscribedSquareCorners;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Commanded Velocity Getters
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Retrieves the last commanded chassis speeds.
     *
     * @return the last commanded ChassisSpeeds (m/s and rad/s)
     */
    public ChassisSpeeds getCommandedVelocities() {
        return commandedVelocities;
    }

    /**
     * Retrieves the last commanded linear velocity as a 2D vector.
     *
     * @return a Vector2d representing the linear velocity (m/s)
     */
    public Vector2d getCommandedLinearVelocity() {
        return new Vector2d(commandedVelocities.vxMetersPerSecond, commandedVelocities.vyMetersPerSecond);
    }

    /**
     * Retrieves the last commanded rotational velocity.
     *
     * @return the rotational velocity in radians per second
     */
    public double getCommandedRotationalVelocity() {
        return commandedVelocities.omegaRadiansPerSecond;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Periodic Update Method
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Periodically updates the drive subsystem.
     * <p>
     * This method is called every cycle (even when disabled) and handles:
     * <ul>
     *   <li>Updating the swerve module states using the current target velocities.</li>
     *   <li>Refreshing the robot's pose estimator.</li>
     *   <li>Updating dynamic pathfinding obstacles based on supplied data.</li>
     * </ul>
     * </p>
     */
    @Override
    public void periodic() {
        // Update module states using the target velocities.
        setModules(targetVelocities);

        // Update the robot's estimated pose using current sensor readings.
        poseEstimator.update(getGyroscopeHeading(), getSwerveModulePositions());

        // Refresh dynamic pathfinding obstacles.
        pathfindingObstacles.clear();
        pathfindingObstaclesSuppliers.forEach(supplier -> pathfindingObstacles.addAll(supplier.get()));
        translatedPathfindingObstacles = translatePathfindingObstacles(pathfindingObstacles);
        Pathfinding.setDynamicObstacles(translatedPathfindingObstacles, getRobotPose().getTranslation());
    }
}

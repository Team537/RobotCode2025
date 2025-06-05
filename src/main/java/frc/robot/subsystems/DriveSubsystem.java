package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Defaults;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants.CoralStationConstants;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Constants.NarwhalConstants;
import frc.robot.Constants.SquidConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveToRotationCommand;
import frc.robot.commands.DriveToTranslationCommand;
import frc.robot.commands.DriveWithRotationalVelocityCommand;
import frc.robot.commands.DriveWithTranslationalVelocityCommand;
import frc.robot.commands.XboxManualDriveCommand;
import frc.robot.util.math.DeltaTime;
import frc.robot.util.swerve.DriveState;
import frc.robot.util.swerve.DrivingMotorType;
import frc.robot.util.autonomous.Alliance;
import frc.robot.util.autonomous.Obstacle;
import frc.robot.util.field.AlgaeRemovalPosition;
import frc.robot.util.field.CoralStationSide;
import frc.robot.util.field.ReefScoringLocation;
import frc.robot.util.swerve.TurningMotorType;
import frc.robot.util.upper_assembly.UpperAssemblyType;

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
    // Component Subsystems
    //////////////////////////////////////////////////////////////////////////////

    public class TranslationalSubsystem extends SubsystemBase {

        // Stores the current target translational velocity in meters per second (robot-relative)
        private Translation2d translationalVelocity = Translation2d.kZero;
    
        /**
         * Returns the current target translational velocity.
         * <p>
         * This represents the desired velocity of the robot in the XY plane,
         * typically set by a command such as autonomous movement or teleop control.
         * </p>
         *
         * @return the robot-relative translational velocity in meters per second
         */
        public Translation2d getTargetTranslationalVelocity() {
            return translationalVelocity;
        }
    
        /**
         * Sets the target translational velocity.
         * <p>
         * This value is usually set continuously by a velocity control command.
         * It represents how fast and in what direction the robot should move along the field.
         * </p>
         *
         * @param velocity the desired robot-relative translational velocity in meters per second
         */
        public void setTargetTranslationalVelocity(Translation2d velocity) {
            translationalVelocity = velocity;
        }
    }
    

    public class RotationalSubsystem extends SubsystemBase {

        // Stores the current target rotational velocity in radians per second (counterclockwise positive)
        private double rotationalVelocity = 0.0;
    
        /**
         * Returns the current target rotational velocity.
         * <p>
         * This represents how quickly the robot should rotate in place.
         * Positive values typically indicate counterclockwise rotation.
         * </p>
         *
         * @return the angular velocity in radians per second
         */
        public double getTargetRotationalVelocity() {
            return rotationalVelocity;
        }
    
        /**
         * Sets the target rotational velocity.
         * <p>
         * This value is typically set continuously by a velocity control command
         * during teleop or autonomous driving.
         * </p>
         *
         * @param velocity the desired angular velocity in radians per second
         */
        public void setTargetRotationalVelocity(double velocity) {
            rotationalVelocity = velocity;
        }
    }

    // Creating the Component Subsystems
    TranslationalSubsystem translationalSubsystem = new TranslationalSubsystem();
    RotationalSubsystem rotationalSubsystem = new RotationalSubsystem();

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
    double[] driveStandardDeviationCoefficients = {
        0.006611986432, 0.3500199104, 0
    };
    double[] visionStandardDeviationCoefficients = { // PLACEHOLDER
        0.0025, 0.0025, 0
    };
    Matrix<N3, N1> driveStandardDeviation = new Matrix<>(N3.instance, N1.instance, driveStandardDeviationCoefficients);
    Matrix<N3, N1> visionStandardDeviation = new Matrix<>(N3.instance, N1.instance, visionStandardDeviationCoefficients);

    /** Gyroscope sensor for obtaining the robot's heading. */
    private Pigeon2 gyroscope = new Pigeon2(DriveConstants.GYROSCOPE_DEVICE_ID);

    /** PID controllers for translational (X and Y) and rotational (theta) control. */
    private PIDController xController = new PIDController(DriveConstants.TRANSLATIONAL_KP, DriveConstants.TRANSLATIONAL_KI, DriveConstants.TRANSLATIONAL_KD);
    private PIDController yController = new PIDController(DriveConstants.TRANSLATIONAL_KP, DriveConstants.TRANSLATIONAL_KI, DriveConstants.TRANSLATIONAL_KD);
    private PIDController thetaController = new PIDController(DriveConstants.ROTATIONAL_KP, DriveConstants.ROTATIONAL_KI, DriveConstants.ROTATIONAL_KD);

    //////////////////////////////////////////////////////////////////////////////
    // Motion and Pose Estimation
    //////////////////////////////////////////////////////////////////////////////

    /** The last chassis speeds actually commanded to the swerve modules. */
    private ChassisSpeeds commandedVelocities = new ChassisSpeeds();

    private Supplier<Boolean> narwhalCanRemoveAlgaeSupplier = (() -> {return true;});

    /**
     * Pose estimator for the robot’s position. Note: Although this is initialized via field
     * declarations, it relies on sensor and module values that have been instantiated above.
     */
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.DRIVE_KINEMATICS,
            getGyroscopeHeading(),
            getSwerveModulePositions(),
            new Pose2d(),
            DriveConstants.DRIVE_STANDARD_DEVIATION,
            VisionConstants.VISION_STANDARD_DEVIATION
    );

    //////////////////////////////////////////////////////////////////////////////
    // Swerve Setpoint Generation
    //////////////////////////////////////////////////////////////////////////////

    /** Generator for swerve drive setpoints based on the current configuration. */
    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint setpoint = new SwerveSetpoint(commandedVelocities, getSwerveModuleStates(), null);
    private DeltaTime setpointDeltaTime = new DeltaTime();

    // private DeltaTime testTime = new DeltaTime(); Commented out due to lack of use.

    //////////////////////////////////////////////////////////////////////////////
    // Motor and Upper Assembly Configuration
    //////////////////////////////////////////////////////////////////////////////

    private DrivingMotorType drivingMotorType = Defaults.DEFAULT_DRIVING_MOTOR;
    private TurningMotorType turningMotorType = Defaults.DEFAULT_TURNING_MOTOR;
    private UpperAssemblyType upperAssemblyType = Defaults.DEFAULT_UPPER_ASSEMBLY;

    //////////////////////////////////////////////////////////////////////////////
    // Pathfinding Configuration
    //////////////////////////////////////////////////////////////////////////////

    private PathConstraints constraints;
    private List<Supplier<List<Obstacle>>> pathfindingObstaclesSuppliers = new ArrayList<>();

    private DriveState state = DriveState.MANUAL;
    private boolean inScorePose = false;
    private boolean inIntakePose = false;
    private boolean inAlgaeRemovePose = false;
    private boolean narwhalCanRaiseLift = false;

    private DeltaTime testTime = new DeltaTime();

    private double translationThreshold;
    private double rotationThreshold;

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
        setPathPlannerConfigs();

        // Pathfinding.setPathfinder(pathfinder);
        // pathfinder.setAvailableTags(AprilTagFields.k2025ReefscapeWelded, DriveConstants.AVAILABLE_SENTINEL_TAGS);
        // pathfinder.setAvailableCameraOffsets(VisionConstants.AVAILABLE_CAMERA_OFFSETS);
        // pathfinder.setWeights(DriveConstants.SENTINEL_DISTANCE_WEIGHT, DriveConstants.SENTINEL_ORIENTATION_WEIGHT);
        // Pathfinding.ensureInitialized();
        PathfindingCommand.warmupCommand();

        // Setup thresholds
        this.translationThreshold = DriveConstants.TRANSLATION_THRESHOLD;
        this.rotationThreshold = DriveConstants.ROTATION_THRESHOLD;
    }

    //////////////////////////////////////////////////////////////////////////////
    // Configuration Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Updates the Path Planner configurations including upper assembly, motor, module, and auto-builder settings.
     */
    public void setPathPlannerConfigs() {

        // IMU Configuration
        // Increase the gyroscope update speed. This puts it in sync with the rest of the code.
        gyroscope.getYaw().setUpdateFrequency(DriveConstants.SENSOR_UPDATE_TIME_HZ);

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
                new PIDConstants(DriveConstants.TRANSLATIONAL_KP, DriveConstants.TRANSLATIONAL_KI, DriveConstants.TRANSLATIONAL_KD),
                new PIDConstants(DriveConstants.ROTATIONAL_KP, DriveConstants.ROTATIONAL_KI, DriveConstants.ROTATIONAL_KD)
        );

        // Calculate maximum translational acceleration using friction and gravity.
        double maxTranslationalAcceleration = DriveConstants.WHEEL_COEFFICIENT_FRICTION * DriveConstants.GRAVITY_ACCELERATION;

        // Determine the effective radius of the drivetrain.
        double effectiveRadius = Math.hypot(DriveConstants.WHEEL_BASE / 2.0, DriveConstants.TRACK_WIDTH / 2.0);

        // Compute maximum rotational velocity and acceleration.
        double maxRotationalVelocity = maxDriveVelocity / effectiveRadius;
        double maxRotationalAcceleration = (DriveConstants.WHEEL_COEFFICIENT_FRICTION * DriveConstants.GRAVITY_ACCELERATION * robotMass * effectiveRadius) / robotMOI;

        // --- Swerve and Auto-Builder Configuration ---
        RobotConfig swerveConfig = new RobotConfig(robotMass, robotMOI, moduleConfig, DriveConstants.MODULE_POSITIONS.toArray(new Translation2d[0]));
        constraints = new PathConstraints(
            DriveConstants.AUTO_DRIVING_TRANSLATIONAL_SPEED_SAFETY_FACTOR * maxDriveVelocity, 
            DriveConstants.AUTO_DRIVING_TRANSLATIONAL_ACCELERATION_SAFETY_FACTOR * maxTranslationalAcceleration, 
            DriveConstants.AUTO_DRIVING_ROTATIONAL_SPEED_SAFETY_FACTOR * maxRotationalVelocity, 
            DriveConstants.AUTO_DRIVING_ROTATIONAL_ACCELERATION_FACTOR * maxRotationalAcceleration);

        AutoBuilder.configure(
                this::getRobotPose,                  // Supplier for current pose
                this::setRobotPose,                  // Consumer to update pose
                this::getVelocity,              // Supplier for chassis speeds
                (chassisSpeeds, feedforward) -> drivePositionalRobotRelative(chassisSpeeds), // Drive command (field-relative)
                controller,                          // Holonomic drive controller
                swerveConfig,                        // Swerve configuration
                () -> false,                         // (Optional) condition for additional behavior
                this
        );

        // --- Setpoint Generator Initialization ---
        setpointGenerator = new SwerveSetpointGenerator(swerveConfig, maxTurningSpeed);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Component Setup Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
    * Sets the default command for the translational sub-subsystem.
    * This command will run whenever no other command is using the translational subsystem.
    *
    * @param command the default command to run
    */
    public void setTranslationalDefaultCommand(Command command) {
        translationalSubsystem.setDefaultCommand(command);
    }

    /**
     * Sets the default command for the rotational sub-subsystem.
     * This command will run whenever no other command is using the rotational subsystem.
     *
     * @param command the default command to run
     */
    public void setRotationalDefaultCommand(Command command) {
        rotationalSubsystem.setDefaultCommand(command);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Command Methods
    //////////////////////////////////////////////////////////////////////////////
    
    @Deprecated
    public Command getManualCommand(XboxController controller, Alliance alliance) {
        return new XboxManualDriveCommand(this, controller, alliance);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Velocity Commands
    //////////////////////////////////////////////////////////////////////////////
    
    // ------------------------------
    // Positional (ChassisSpeeds) Commands
    // ------------------------------

    /**
     * Creates a command that drives the robot using both translational and rotational velocity.
     * These velocities are supplied as a full ChassisSpeeds object.
     * <p>
     * The command runs two sub-commands in parallel:
     * - One for handling field-relative translational movement (vx, vy)
     * - One for handling rotational velocity (omega)
     * </p>
     * 
     * @param velocitySupplier a supplier for ChassisSpeeds representing desired velocities (field-relative)
     * @return a command that drives the robot using those velocities
     */
    public Command getPositionalVelocityCommand(Supplier<ChassisSpeeds> velocitySupplier) {
        return new ParallelCommandGroup(
            getTranslationalVelocityCommand(() -> 
                new Translation2d(
                    velocitySupplier.get().vxMetersPerSecond,
                    velocitySupplier.get().vyMetersPerSecond
                )
            ),
            getRotationalVelocityCommand(() -> 
                velocitySupplier.get().omegaRadiansPerSecond
            )
        );
    }

    /**
     * Creates a command that drives the robot using a fixed, field-relative translational and rotational velocity.
     *
     * @param velocity the fixed field-relative chassis speeds (vx, vy, omega)
     * @return a command that drives the robot at the specified field-relative velocity
     */
    public Command getPositionalVelocityCommand(ChassisSpeeds velocity) {
        return getPositionalVelocityCommand(() -> velocity);
    }

    /**
     * Creates a command that drives the robot using zero field-relative translational and rotational velocity.
     * Useful for stopping the robot using the same structure as velocity-based commands.
     *
     * @return a command that stops the robot (field-relative)
     */
    public Command getPositionalStopCommand() {
        return getPositionalVelocityCommand(new ChassisSpeeds());
    }

    /**
     * Creates a command that drives the robot using both translational and rotational velocity.
     * These velocities are supplied as a full ChassisSpeeds object, interpreted as robot-relative.
     * <p>
     * The command runs two sub-commands in parallel:
     * - One for handling robot-relative translational movement (vx, vy)
     * - One for handling rotational velocity (omega)
     * </p>
     *
     * @param velocitySupplier a supplier for ChassisSpeeds representing desired velocities (robot-relative)
     * @return a command that drives the robot using those robot-relative velocities
     */
    public Command getRobotRelativePositionalVelocityCommand(Supplier<ChassisSpeeds> velocitySupplier) {
        return new ParallelCommandGroup(
            getRobotRelativeTranslationalVelocityCommand(() -> 
                new Translation2d(
                    velocitySupplier.get().vxMetersPerSecond,
                    velocitySupplier.get().vyMetersPerSecond
                )
            ),
            getRotationalVelocityCommand(() -> 
                velocitySupplier.get().omegaRadiansPerSecond
            )
        );
    }

    /**
     * Creates a command that drives the robot using a fixed, robot-relative translational and rotational velocity.
     *
     * @param velocity the fixed robot-relative chassis speeds (vx, vy, omega)
     * @return a command that drives the robot at the specified robot-relative velocity
     */
    public Command getRobotRelativePositionalVelocityCommand(ChassisSpeeds velocity) {
        return getRobotRelativePositionalVelocityCommand(() -> velocity);
    }

    // ------------------------------
    // Translational (Translation2d) Commands
    // ------------------------------

    /**
     * Creates a command to control translational velocity in a field-relative way.
     * This does not affect the robot’s rotation.
     *
     * @param velocitySupplier supplier for desired translational velocity in field coordinates (m/s)
     * @return a command that drives the robot translationally
     */
    public Command getTranslationalVelocityCommand(Supplier<Translation2d> velocitySupplier) {
        Command command = new DriveWithTranslationalVelocityCommand(this, velocitySupplier);
        command.addRequirements(translationalSubsystem); // ensures no other command can use this subsystem concurrently
        return command;
    }

    /**
     * Creates a command to drive the robot at a constant field-relative translational velocity.
     * This does not affect the robot’s rotation.
     *
     * @param velocity a constant field-relative translation vector (m/s)
     * @return a command that drives the robot translationally at the given velocity
     */
    public Command getTranslationalVelocityCommand(Translation2d velocity) {
        return getTranslationalVelocityCommand(() -> velocity);
    }

    /**
     * Creates a command that drives the robot translationally with zero velocity (stopped).
     *
     * @return a command that stops the robot's translation
     */
    public Command getTranslationalStoppedCommand() {
        return getTranslationalVelocityCommand(Translation2d.kZero);
    }

    /**
     * Creates a command to control translational velocity in a robot-relative way.
     * Converts robot-relative translation to field-relative using current robot heading.
     *
     * @param robotRelativeVelocitySupplier supplier of robot-relative Translation2d (m/s)
     * @return a command that drives the robot translationally based on robot-relative input
     */
    public Command getRobotRelativeTranslationalVelocityCommand(Supplier<Translation2d> robotRelativeVelocitySupplier) {
        Command command = new DriveWithTranslationalVelocityCommand(
            this,
            () -> robotRelativeVelocitySupplier.get().rotateBy(getRobotPose().getRotation()) // convert to field-relative
        );
        command.addRequirements(translationalSubsystem); // lock the translational subsystem
        return command;
    }

    /**
     * Creates a command to drive the robot at a constant robot-relative translational velocity.
     * Converts the input to field-relative using the robot’s current heading at runtime.
     *
     * @param robotRelativeVelocity a constant robot-relative translation vector (m/s)
     * @return a command that drives the robot at the given robot-relative velocity
     */
    public Command getRobotRelativeTranslationalVelocityCommand(Translation2d robotRelativeVelocity) {
        return getRobotRelativeTranslationalVelocityCommand(() -> robotRelativeVelocity);
    }

    // ------------------------------
    // Rotational (Double) Commands
    // ------------------------------

    /**
     * Creates a command to control rotational velocity.
     * This does not affect the robot’s translational movement.
     *
     * @param velocitySupplier supplier for desired rotational velocity in radians per second
     * @return a command that drives the robot rotationally
     */
    public Command getRotationalVelocityCommand(Supplier<Double> velocitySupplier) {
        Command command = new DriveWithRotationalVelocityCommand(this, velocitySupplier);
        command.addRequirements(rotationalSubsystem); // ensures exclusive access to rotation control
        return command;
    }

    /**
     * Creates a command to drive the robot at a constant rotational velocity.
     * This does not affect the robot’s translational movement.
     *
     * @param velocity a fixed rotational velocity in radians per second
     * @return a command that drives the robot rotationally at the given velocity
     */
    public Command getRotationalVelocityCommand(double velocity) {
        return getRotationalVelocityCommand(() -> velocity);
    }

    /**
     * Creates a command that drives the robot with zero rotational velocity (stopped).
     * This does not affect the robot’s translational movement.
     *
     * @return a command that stops the robot’s rotation
     */
    public Command getRotationalStoppedCommand() {
        return getRotationalVelocityCommand(0.0);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Targeting Commands
    //////////////////////////////////////////////////////////////////////////////
    
    // ------------------------------
    // Positional (Pose2d) Commands
    // ------------------------------

    /**
     * Generates a command that drives the robot to a specific pose (XY and heading).
     * The command finishes once both the translation and rotation are within their thresholds.
     *
     * @param pose the target Pose2d to reach
     * @return a Command that drives to the specified pose
     */
    public Command getDriveToPoseCommand(Pose2d pose) {
        return getDriveToPoseCommand(() -> pose);
    }

    /**
     * Generates a command that drives the robot to a pose dynamically supplied at runtime.
     * Both translation and rotation targets are extracted from the pose returned by the supplier.
     *
     * @param poseSupplier supplies the target Pose2d during execution
     * @return a Command that drives to the supplied pose target
     */
    public Command getDriveToPoseCommand(Supplier<Pose2d> poseSupplier) {
        return getDriveToTranslationCommand(() -> poseSupplier.get().getTranslation())
            .alongWith(getDriveToRotationCommand(() -> poseSupplier.get().getRotation()));
    }

    // ------------------------------
    // Positional (Translation2d) Commands
    // ------------------------------

    /**
     * Generates a command that drives the robot to a specific translation (XY position).
     * The robot will attempt to reach and stop at the given fixed translation.
     *
     * @param translation the target Translation2d to reach
     * @return a Command that drives to the specified translation
     */
    public Command getDriveToTranslationCommand(Translation2d translation) {
        return getDriveToTranslationCommand(() -> translation);
    }

    /**
     * Generates a command that drives the robot to a dynamically provided translation.
     * The supplier will be polled continuously while the command is active.
     *
     * @param translationSupplier supplies the target Translation2d during execution
     * @return a Command that drives to the supplied translation target
     */
    public Command getDriveToTranslationCommand(Supplier<Translation2d> translationSupplier) {
        return new DriveToTranslationCommand(this, translationSupplier, translationThreshold);
    }

    // ------------------------------
    // Rotational (Rotation2d) Commands
    // ------------------------------

    /**
     * Generates a command that rotates the robot to a specific heading (theta).
     * The robot will turn until it is within a defined threshold of the fixed rotation.
     *
     * @param rotation the target Rotation2d to reach
     * @return a Command that rotates the robot to the specified heading
     */
    public Command getDriveToRotationCommand(Rotation2d rotation) {
        return getDriveToRotationCommand(() -> rotation);
    }

    /**
     * Generates a command that rotates the robot toward a heading provided by a supplier.
     * The supplier will be evaluated continuously while the command is running.
     *
     * @param rotationSupplier supplies the target Rotation2d during execution
     * @return a Command that rotates the robot to the supplied heading
     */
    public Command getDriveToRotationCommand(Supplier<Rotation2d> rotationSupplier) {
        return new DriveToRotationCommand(this, rotationSupplier, rotationThreshold);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Pathfinding Commands
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Generates a command that uses pathfinding to drive the robot to a specified pose.
     *
     * @param pose the target Pose2d to reach
     * @return a Command that, when executed, will pathfind to the target pose
     */
    public Command getPathfindingCommand(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, constraints, 0.0)
            .andThen(getDriveToPoseCommand(pose));
    }

    //////////////////////////////////////////////////////////////////////////////
    // Action Commands
    //////////////////////////////////////////////////////////////////////////////

    // ------------------------------
    // Scoring Command
    // ------------------------------


    /**
     * Creates a scoring command using a specific scoring location.
     *
     * @param alliance the alliance (e.g. Alliance.BLUE or Alliance.RED)
     * @param location the ReefScoringLocation (for example, A, B, C, etc.)
     * @return a Command that will drive the robot to the scoring pose.
     */
    public Command getScoringCommand(Alliance alliance, ReefScoringLocation location) {

        Pose2d targetPose = ReefConstants.CORAL_SCORE_POSITIONS.get(alliance).get(location);

        if (upperAssemblyType == UpperAssemblyType.NARWHAL || upperAssemblyType == UpperAssemblyType.NONE) {
            targetPose = targetPose.transformBy(NarwhalConstants.SCORING_RELATIVE_TRANSFORM);
        }

        System.out.println("Going to drive to position ("+ targetPose.getX() + ", " + targetPose.getY() + ')');
        return 
            (
                new InstantCommand(() -> {state = DriveState.SCORING;inScorePose = false;})
            ).andThen(
                wrapNarwhalLiftRaising(getPathfindingCommand(targetPose),targetPose) 
            ).andThen(
                new InstantCommand(() -> {inScorePose = true;})
            );
            
    }

    // ------------------------------
    // Algae Removal Command
    // ------------------------------

    /**
     * Creates an algae removal command using a specific algae removal position.
     *
     * @param alliance the alliance (e.g. Alliance.BLUE or Alliance.RED)
     * @param position the AlgaeRemovalPosition (for example, AB, CD, EF, GH, IJ, or KL)
     * @return a Command that will drive the robot to the algae removal pose.
     */
    public Command getAlgaeRemovalCommand(Alliance alliance, AlgaeRemovalPosition position) {
        
        Pose2d targetPose = ReefConstants.ALGAE_REMOVAL_POSITIONS.get(alliance).get(position);

        Command moveAfterUpperAssemblyCommand;
        if (upperAssemblyType == UpperAssemblyType.NARWHAL || upperAssemblyType == UpperAssemblyType.NONE) {
            targetPose = targetPose.transformBy(NarwhalConstants.ALGAE_REMOVAL_RELATIVE_TRANSFORM);
            moveAfterUpperAssemblyCommand = new WaitUntilCommand(narwhalCanRemoveAlgaeSupplier::get)
                .andThen(getPathfindingCommand(targetPose.transformBy(DriveConstants.NARWHAL_RAKE_ALGAE_TRANSFORM)));
        } else {
            moveAfterUpperAssemblyCommand = new InstantCommand();
        }

        return 
            (
                new InstantCommand(() -> {state = DriveState.REMOVING;inAlgaeRemovePose = false;})
            ).andThen(
                wrapNarwhalLiftRaising(getPathfindingCommand(targetPose),targetPose) 
            ).andThen(
                new InstantCommand(() -> {inAlgaeRemovePose = true;})
            ).andThen(
                moveAfterUpperAssemblyCommand
            );
    }

    // ------------------------------
    // Intake Command
    // ------------------------------

    /**
     * Creates an intake command using a specific CoralStationSide and slot index.
     *
     * @param alliance the alliance (e.g. Alliance.BLUE or Alliance.RED)
     * @param side the CoralStationSide (for example, LEFT or RIGHT)
     * @param slot the slot index (an int used to pick from a pre-defined list of intake poses)
     * @return a Command that will drive the robot to the intake pose.
     */
    public Command getIntakeCommand(Alliance alliance, CoralStationSide side, int slot) {
        Pose2d targetPose;
        // For intake we assume that you have lists (or arrays) of intake positions defined in ReefConstants.
        if (alliance == Alliance.BLUE) {
            if (side == CoralStationSide.LEFT) {
                targetPose = CoralStationConstants.BLUE_CORAL_INTAKE_LEFT.get(slot);
            } else if (side == CoralStationSide.RIGHT) {
                targetPose = CoralStationConstants.BLUE_CORAL_INTAKE_RIGHT.get(slot);
            } else {
                throw new IllegalArgumentException("Invalid CoralStationSide: " + side);
            }
        } else { // Alliance.RED
            if (side == CoralStationSide.LEFT) {
                targetPose = CoralStationConstants.RED_CORAL_INTAKE_LEFT.get(slot);
            } else if (side == CoralStationSide.RIGHT) {
                targetPose = CoralStationConstants.RED_CORAL_INTAKE_RIGHT.get(slot);
            } else {
                throw new IllegalArgumentException("Invalid CoralStationSide: " + side);
            }
        }

        if (upperAssemblyType == UpperAssemblyType.NARWHAL || upperAssemblyType == UpperAssemblyType.NONE) {
            targetPose = targetPose.transformBy(NarwhalConstants.INTAKING_RELATIVE_TRANSFORM);
        }


        return 
            (
                new InstantCommand(() -> {state = DriveState.INTAKING;inIntakePose = false;})
            ).andThen(
                wrapNarwhalLiftRaising(getPathfindingCommand(targetPose),targetPose) 
            ).andThen(
                new InstantCommand(() -> {inIntakePose = true;})
            );
    }

    //////////////////////////////////////////////////////////////////////////////
    // Helper Commands
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Wraps a command with logic for controlling narwhal lift activation.
     * Prevents the lift from raising until the robot is near the target.
     *
     * @param command the core command (e.g., pathfinding or driving)
     * @param pose the target pose to compare distance against
     * @return a wrapped command with narwhal lift logic
     */
    private Command wrapNarwhalLiftRaising(Command command, Pose2d pose) {
        return new InstantCommand(() -> narwhalCanRaiseLift = false)
            .andThen(command)
            .deadlineFor(
                new RunCommand(() -> {
                    narwhalCanRaiseLift =
                        getRobotPose().getTranslation().getDistance(pose.getTranslation())
                        <= DriveConstants.NARWHAL_CAN_RAISE_LIFT_DISTANCE;
                })
            )
            .andThen(new InstantCommand(() -> narwhalCanRaiseLift = true));
    }

    //////////////////////////////////////////////////////////////////////////////
    // Driving Command Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Commands the robot to drive with both translational and rotational velocity in a field-relative manner.
     *
     * @param velocity the desired velocities, where dx/dy are in meters per second and dtheta is in radians per second.
     */
    public void drivePositional(ChassisSpeeds velocity) {
        driveTranslational(new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond)); // field-relative translational
        driveRotational(velocity.omegaRadiansPerSecond);                             // rotational velocity
    }

    /**
     * Commands the robot to drive with a field-relative translational velocity.
     *
     * @param velocity the desired field-relative translational velocity (m/s)
     */
    public void driveTranslational(Translation2d velocity) {
        translationalSubsystem.setTargetTranslationalVelocity(velocity);
    }

    /**
     * Commands the robot to rotate at a specified rotational velocity.
     *
     * @param velocity the desired rotational velocity in radians per second
     */
    public void driveRotational(double velocity) {
        rotationalSubsystem.setTargetRotationalVelocity(velocity);
    }

    /**
     * Commands the robot to drive with both translational and rotational velocity in a robot-relative manner.
     *
     * @param velocity the desired robot-relative velocities (dx, dy in m/s; dtheta in rad/s)
     */
    public void drivePositionalRobotRelative(ChassisSpeeds velocity) {
        Rotation2d robotHeading = getRobotPose().getRotation();
        drivePositional(ChassisSpeeds.fromFieldRelativeSpeeds(velocity, robotHeading));
    }
    
    /**
     * Commands the robot to drive with a translational velocity relative to the robot's current orientation.
     *
     * @param velocity the desired robot-relative translational velocity (m/s)
     */
    public void driveTranslationalRobotRelative(Translation2d velocity) {
        Rotation2d robotHeading = getRobotPose().getRotation();
        Translation2d fieldRelativeVelocity = velocity.rotateBy(robotHeading);
        driveTranslational(fieldRelativeVelocity);
    }

    //////////////////////////////////////////////////////////////////////////////
    // Feedback and Navigation Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Returns the translational velocity value from the PID controllers, as a Vector2d.
     * 
     * @param target The target position.
     * @return The translational velocity, as a Vector2d.
     */
    public Translation2d getTranslationalFeedback(Translation2d target) {
        Translation2d feedback = new Translation2d(
                xController.calculate(getRobotPose().getX(), target.getX()),
                yController.calculate(getRobotPose().getY(), target.getY())
        );
        return feedback.div(feedback.getNorm()).times(Math.tanh(feedback.getNorm()));
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
     * Set the translational and rotational thresholds.
     * 
     * @param translationalThreshold The translational threshold.
     * @param rotationalThreshold The rotational threshold.
     */
     public void setThresholds(double translationalThreshold, double rotationalThreshold) {
        this.translationThreshold = translationalThreshold;
        this.rotationThreshold = rotationalThreshold;
     }

    //////////////////////////////////////////////////////////////////////////////
    // Upper Assembly Helped Methods
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Returns whether the robot has reached the scoring pose.
     * This flag is true only when the robot is in the scoring pose and
     * the drive state is {@code DriveState.SCORING}.
     *
     * @return {@code true} if the robot is in the scoring pose and the state is SCORING, {@code false} otherwise.
     */
    public boolean getInScorePose() {
        return inScorePose && state == DriveState.SCORING;
    }

    /**
     * Returns whether the robot has reached the algae removal pose.
     * This flag is true only when the robot is in the algae removal pose and
     * the drive state is {@code DriveState.REMOVING}.
     *
     * @return {@code true} if the robot is in the algae removal pose and the state is REMOVING, {@code false} otherwise.
     */
    public boolean getInRemovePose() {
        return inAlgaeRemovePose && state == DriveState.REMOVING;
    }

    /**
     * Returns whether the robot has reached the intake pose.
     * This flag is true only when the robot is in the intake pose and
     * the drive state is {@code DriveState.INTAKING}.
     *
     * @return {@code true} if the robot is in the intake pose and the state is INTAKING, {@code false} otherwise.
     */
    public boolean getInIntakePose() {
        return inIntakePose && state == DriveState.INTAKING;
    }

    /**
     * Provides additional information for the Narwhal upper assembly.
     * This method indicates whether the lift can be raised without risking tipping.
     * The value is typically used as a safeguard to ensure that the lift is raised
     * only when the robot is near the target pose.
     *
     * @return {@code true} if the narwhal upper assembly is allowed to raise the lift, {@code false} otherwise.
     */
    public boolean getNarwhalCanRaiseLift() {
        return narwhalCanRaiseLift;
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
     * Returns this DriveSubsystem's swerver modules' states.
     * 
     * @return this DriveSubsystem's swerver modules' states.
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
     * Retrieves the current chassis speeds computed from the module states.
     *
     * @return the chassis speeds (m/s and rad/s)
     */
    public ChassisSpeeds getVelocity() {
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
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
     * Sets the turning motor of all four swerve modules.
     * 
     * @param turningMotor The turning motor to set.
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

    /**
     * sets the supplier for whether the drivetrain can move to remove algae from the reef
     * @param supplier the supplier, that when true, will move the drivetrain.
     */
    public void setNarwhalCanRemoveAlgaeSupplier(Supplier<Boolean> supplier) {
        narwhalCanRemoveAlgaeSupplier = supplier;
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
    // Setting Modules Method
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Updates the states of the swerve modules based on the current target velocities.
     *
     * @param targetVelocity the desired field relative chassis speeds (m/s and rad/s)
     */
    private void setModules(ChassisSpeeds targetVelocity) {
        // Generate a new setpoint from the previous one.
        setpoint = setpointGenerator.generateSetpoint(setpoint, ChassisSpeeds.fromFieldRelativeSpeeds(targetVelocity, getRobotPose().getRotation()), setpointDeltaTime.getDeltaTime());

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
    // Periodic Update Method
    //////////////////////////////////////////////////////////////////////////////

    /**
     * Updates the robot's odometry. This is done via a separate method to allow for faster update times
     * than the base WPILib periodic method.
     */
    public void updateOdometry() {

        // Update the robot's estimated pose using current sensor readings.
        poseEstimator.update(getGyroscopeHeading(), getSwerveModulePositions());

        // Display the data 
        SmartDashboard.putNumber("X Position",getRobotPose().getX());
        SmartDashboard.putNumber("Y Position",getRobotPose().getY());
        SmartDashboard.putNumber("Theta Rotation", getRobotPose().getRotation().getRadians());
    }

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
        ChassisSpeeds targetVelocity = new ChassisSpeeds(
            translationalSubsystem.getTargetTranslationalVelocity().getX(),
            translationalSubsystem.getTargetTranslationalVelocity().getY(),
            rotationalSubsystem.getTargetRotationalVelocity()
        );
        setModules(targetVelocity);

        // Refresh dynamic pathfinding obstacles.
        List<Obstacle> pathfindingObstacles = new ArrayList<>();
        pathfindingObstaclesSuppliers.forEach(supplier -> pathfindingObstacles.addAll(supplier.get()));
        List<Pair<Translation2d,Translation2d>> translatedPathfindingObstacles = translatePathfindingObstacles(pathfindingObstacles);
        //Pathfinding.setDynamicObstacles(translatedPathfindingObstacles, getRobotPose().getTranslation());
    }

    @Override
    public void simulationPeriodic() {
        setRobotPose(getRobotPose().exp(ChassisSpeeds.fromFieldRelativeSpeeds(commandedVelocities, getRobotPose().getRotation()).toTwist2d(testTime.getDeltaTime())));
    }

}

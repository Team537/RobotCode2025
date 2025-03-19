package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.autonomous.Alliance;
import frc.robot.util.math.Vector2d;

public abstract class ManualDriveCommand extends Command {
    
    protected final DriveSubsystem driveSubsystem;
    protected final XboxController controller;
    protected final Alliance alliance;

    /**
     * The rotational offset of the driver
     */
    private Rotation2d driverRotationalOffset = new Rotation2d(0.0);

    /**
     * Indicates whether target translation is active.
    */
    private boolean targetTranslationActive = false;

    /**
     * The origin point for target translation calculations.
     */
    private Translation2d targetTranslationOrigin = new Translation2d(0.0, 0.0);

    /**
     * If the robot is locked in the X-Y plane (i.e. holding position).
     */
    private boolean xyLockActive = false;

    /**
     * The translation where the robot is locked.
     */
    private Translation2d xyLockTranslation = new Translation2d(0.0, 0.0);

    /**
     * Tracks whether linear velocity has been reset.
     */
    private boolean linearVelocityReset = false;

    /**
     * If the robot's heading is locked.
     */
    private boolean thetaLockActive = false;

    /**
     * The rotation (angle) where the robot is locked.
     */
    private Rotation2d thetaLockRotation = new Rotation2d(0.0);
    
    /**
     * Tracks whether rotational velocity has been reset.
     */
    private boolean rotationalVelocityReset = false;

    /**
     * Indicates whether target rotation is active.
     */
    private boolean targetRotationActive = false;

    /**
     * The origin angle for target rotation calculations.
     */
    private Rotation2d targetRotationOrigin = new Rotation2d(0.0);

    /**
     * Creates a new manual drive command to allow for the robot to be controlled manually during teleop.
     * 
     * @param driveSubsystem The robot's drive subsystem.
     * @param controller The controller used by the driver.
     */
    public ManualDriveCommand(DriveSubsystem driveSubsystem, XboxController controller, Alliance alliance) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        this.alliance = alliance;
        addRequirements(driveSubsystem);
        
        driverRotationalOffset = alliance.getDriverRotationalOffset();
    }

    /**
     * Drives the robot using manual inputs or target-based controls. This method calculates 
     * the final linear and rotational velocities based on the input parameters and applies 
     * them to the robot.
     *
     * @param linearVelocity The target linear velocity of the robot. Used when useTargetTranslation is false.
     *                       This is represented as a vector on a unit circle.
     * @param rotationalVelocity The target rotational velocity of the robot. Used when useTargetRotation is false.
     *                           This is a unit value (range [-1, 1]).
     * @param targetTranslationOffset The desired translation offset for targeting a specific position.
     * @param useTargetTranslation Whether or not the robot should aim for a target translation.
     * @param targetRotationOffset The desired rotation offset for targeting a specific orientation 
     *                              (or the absolute orientation if useAbsoluteRotation is true). Measured in radians.
     * @param useTargetRotation Whether or not the robot should aim for a target rotation.
     * @param useAbsoluteRotation Whether the robot should target an absolute orientation or a relative offset.
     * @param throttle A multiplier for controlling the max speed of the robot. Ranges from 0 (minimum speed) to 1 (maximum speed).
     * @param slow
     * @param fieldCentric
     */
    protected void manualDrive(
        Vector2d linearVelocity, 
        double rotationalVelocity, 
        Translation2d targetTranslationOffset, 
        boolean useTargetTranslation, 
        Rotation2d targetRotationOffset, 
        boolean useTargetRotation, 
        boolean useAbsoluteRotation, 
        double throttle,
        double slow,
        boolean fieldCentric
    ) {
        // The final velocities that will be used for driving the robot
        Vector2d finalLinearVelocity;
        double finalRotationalVelocity;

        // --- Handling linear velocity ---
        if (!useTargetTranslation) {
            // Target translation is not active, so use joystick input for linear control
            targetTranslationActive = false;

            // Rotate and curve the input for smoother control
            if (fieldCentric) {
                linearVelocity = linearVelocity.rotateBy(driverRotationalOffset.times(1.0)); // Adjust for driver orientation
            } else {
                linearVelocity = linearVelocity.rotateBy(driveSubsystem.getRobotPose().getRotation().times(1.0).rotateBy(new Rotation2d(0.5 * Math.PI)));
            }

            double linearSpeed = linearVelocity.magnitude();
            linearVelocity = linearVelocity.normalize().scale(Math.pow(linearSpeed,OperatorConstants.LINEAR_INPUT_CURVE_POWER));

            // Resetting linear velocity when no input is provided
            if (linearVelocity.magnitude() < 1e-3) {
                linearVelocityReset = true;
            }
            if (!linearVelocityReset) {
                linearVelocity = new Vector2d(0, 0);
            }

            // TODO: test and fix this, delete the "false && " to activate
            if (linearVelocity.magnitude() < 1e-3) {
                
                if (!xyLockActive && driveSubsystem.getCommandedLinearVelocity().magnitude() < 1e-3) {
                    xyLockActive = true;
                    xyLockTranslation = driveSubsystem.getRobotPose().getTranslation();
                }

                if (false && xyLockActive) {
                    finalLinearVelocity = driveSubsystem.getLinearFeedback(xyLockTranslation).scale(DriveConstants.LINEAR_MAX_SPEED);
                } else {
                    finalLinearVelocity = new Vector2d(0.0,0.0);
                }
                
            } else {

                xyLockActive = false;

                // Apply throttle for speed control
                double linearThrottleMultiplier;
                if (slow < 1e-2) {
                    linearThrottleMultiplier = OperatorConstants.NORMAL_LINEAR_MAX_SPEED + throttle * 
                        (OperatorConstants.THROTTLE_LINEAR_MAX_SPEED - OperatorConstants.NORMAL_LINEAR_MAX_SPEED);
                } else {
                    linearThrottleMultiplier = OperatorConstants.NORMAL_LINEAR_MAX_SPEED + slow * 
                        (OperatorConstants.SLOW_LINEAR_MAX_SPEED - OperatorConstants.NORMAL_LINEAR_MAX_SPEED);
                }
                
                finalLinearVelocity = linearVelocity.scale(linearThrottleMultiplier);
            }

        } else {

            xyLockActive = false;

            // Target translation is active
            if (fieldCentric) {
                targetTranslationOffset = targetTranslationOffset.rotateBy(driverRotationalOffset.times(1.0));
            } else {
                targetTranslationOffset = targetTranslationOffset.rotateBy(driveSubsystem.getRobotPose().getRotation().times(1.0).rotateBy(new Rotation2d(0.5 * Math.PI)));
            }
            
            if (!targetTranslationActive) {

                // Initialize the origin for target translation
                targetTranslationActive = true;
                targetTranslationOrigin = driveSubsystem.getRobotPose().getTranslation().minus(targetTranslationOffset);
            }

            // Use PID feedback to calculate the velocity toward the target position
            finalLinearVelocity = driveSubsystem.getLinearFeedback(targetTranslationOrigin.plus(targetTranslationOffset)).scale(DriveConstants.LINEAR_MAX_SPEED);
            linearVelocityReset = false; // Prevent accidental movement when deactivating
        }

        // --- Handling rotational velocity ---
        if (!useTargetRotation) {

            // Target rotation is not active, so use joystick input for rotation control
            targetRotationActive = false;

            // Curve the rotational input for smoother control
            double rotationalSpeed = Math.abs(rotationalVelocity);
            rotationalVelocity = Math.signum(rotationalVelocity) * Math.pow(rotationalSpeed, OperatorConstants.ROTATION_INPUT_CURVE_POWER);

            // Resetting rotational velocity when no input is provided
            if (Math.abs(rotationalVelocity) < 1e-3) {
                rotationalVelocityReset = true;
            }
            if (!rotationalVelocityReset) {
                rotationalVelocity = 0.0;
            }

            // Lock the robot's orientation if no rotation is commanded

            if (Math.abs(rotationalVelocity) < 1e-3) {
                if (!thetaLockActive && Math.abs(driveSubsystem.getCommandedRotationalVelocity()) < 1e-3) {
                    thetaLockActive = true;
                    thetaLockRotation = driveSubsystem.getRobotPose().getRotation();
                }
                
                if (false && thetaLockActive) {
                    finalRotationalVelocity = driveSubsystem.getRotationalFeedback(thetaLockRotation) * DriveConstants.ROTATIONAL_MAX_SPEED;
                } else {
                    finalRotationalVelocity = 0.0;
                }
            } else {

                thetaLockActive = false;

                double rotationalThrottleMultiplier;
                if (slow < 1e-2) {
                    rotationalThrottleMultiplier = OperatorConstants.NORMAL_ROTATIONAL_MAX_SPEED + throttle * 
                        (OperatorConstants.THROTTLE_ROTATIONAL_MAX_SPEED - OperatorConstants.NORMAL_ROTATIONAL_MAX_SPEED);
                } else {
                    rotationalThrottleMultiplier = OperatorConstants.NORMAL_ROTATIONAL_MAX_SPEED + slow * 
                        (OperatorConstants.SLOW_ROTATIONAL_MAX_SPEED - OperatorConstants.NORMAL_ROTATIONAL_MAX_SPEED);
                }
                finalRotationalVelocity = rotationalVelocity * rotationalThrottleMultiplier;
            }

        } else {

            // Target rotation is active
            targetRotationOffset.rotateBy(driverRotationalOffset.times(1.0)); // Adjust for driver orientation

            if (!useAbsoluteRotation) {
                // Targeting a relative rotation
                if (!targetRotationActive) {
                    // Initialize the origin for target rotation
                    targetRotationActive = true;
                    targetRotationOrigin = driveSubsystem.getRobotPose().getRotation().minus(targetRotationOffset);
                }
                // Use PID feedback to calculate the rotational velocity
                finalRotationalVelocity = driveSubsystem.getRotationalFeedback(targetRotationOrigin.rotateBy(targetRotationOffset)) * DriveConstants.ROTATIONAL_MAX_SPEED;
            } else {
                // Targeting an absolute rotation
                targetRotationActive = false; // Disable relative rotation targeting
                finalRotationalVelocity = driveSubsystem.getRotationalFeedback(targetRotationOffset) * DriveConstants.ROTATIONAL_MAX_SPEED;
            }

            rotationalVelocityReset = false; // Prevent accidental movement when deactivating
        }

        // --- Apply the calculated velocities to the robot ---
        driveSubsystem.drive(new ChassisSpeeds(finalLinearVelocity.getX(),finalLinearVelocity.getY(),finalRotationalVelocity));
    }

}
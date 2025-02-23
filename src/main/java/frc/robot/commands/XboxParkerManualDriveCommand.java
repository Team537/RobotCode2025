package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Vector2d;

public class XboxParkerManualDriveCommand extends ManualDriveCommand {

    boolean orientationOffsetTargetActive = false;

    /**
     * Creates a manual drive command for Parker, using the button mappings and functionality of the XBox controller.
     * 
     * @param driveSubsystem The robot's drive subsystem.
     * @param controller The controller used by the driver.
     */
    public XboxParkerManualDriveCommand(DriveSubsystem driveSubsystem, XboxController controller) {
        super(driveSubsystem,controller);
    }

    @Override
    public void execute() {

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
            (orientationOffsetTargetActive && rightStickVector.magnitude() > OperatorConstants.XBOX_CONTROLLER_ROTATIONAL_TARGET_DEACTIVATION_ZONE) // Stay active if above threshold
        ) {
            orientationOffsetTargetActive = true; // Enable rotation targeting
        } else {
            orientationOffsetTargetActive = false; // Disable rotation targeting
        }
        boolean useTargetRotation = orientationOffsetTargetActive;

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

}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.math.Vector2d;

/**
 * Command to drive the robot to a specified pose.
 * <p>
 * This command uses the drive subsystem’s feedback methods to calculate
 * linear and rotational speeds that drive the robot toward a target pose.
 * The command finishes when the robot’s translation is within a specified
 * threshold and its heading error is small. When finished, the robot is stopped.
 * </p>
 */
public class DriveToPoseCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPose;
    private final double translationThreshold; // in meters
    private final double rotationThreshold;    // in radians

    /**
     * Constructs a new DriveTotranslationCommand.
     *
     * @param driveSubsystem    the drive subsystem used to drive the robot
     * @param targetPose        the target pose (translation and orientation) to drive to
     * @param translationThreshold the allowable error (in meters) before the command finishes
     * @param rotationThreshold the allowable error (in radians) before the command finishes
     */
    public DriveToPoseCommand(DriveSubsystem driveSubsystem, Pose2d targetPose, double translationThreshold, double rotationThreshold) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;
        this.translationThreshold = translationThreshold;
        this.rotationThreshold = rotationThreshold;
        addRequirements(driveSubsystem);
    }

    /**
     * Called repeatedly while the command is scheduled.
     * <p>
     * Computes the linear and rotational feedback required to drive toward the target pose,
     * and commands the drive subsystem accordingly.
     * </p>
     */
    @Override
    public void execute() {

        // Calculate the drive feedback.
        // getLinearFeedback returns a Vector2d (with X and Y velocities) based on the translation error.
        Vector2d linearFeedback = driveSubsystem.getLinearFeedback(targetPose.getTranslation());
        // getRotationalFeedback returns a rotational speed based on the heading error.
        double rotationalFeedback = driveSubsystem.getRotationalFeedback(targetPose.getRotation());

        // Create chassis speeds from the computed feedback.
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                linearFeedback.getX(),
                linearFeedback.getY(),
                rotationalFeedback
        );

        // Command the drive subsystem.
        driveSubsystem.drive(chassisSpeeds);
    }

    /**
     * Returns whether the command has finished.
     * <p>
     * The command finishes when both the translation error (distance) and the heading error
     * are below the specified thresholds.
     * </p>
     *
     * @return {@code true} if the robot is within the desired threshold; {@code false} otherwise.
     */
    @Override
    public boolean isFinished() {
        Pose2d currentPose = driveSubsystem.getRobotPose();
        double distanceError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        System.out.println(rotationError);
        return (distanceError < translationThreshold) && (rotationError < rotationThreshold);
    }

    /**
     * Called once the command ends or is interrupted.
     * <p>
     * This method stops the robot by commanding zero speeds.
     * </p>
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the robot.
        driveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

/**
 * Command to rotate the robot to a dynamically supplied heading.
 * <p>
 * This command uses a supplier to obtain the target heading on each loop.
 * The drive subsystem provides feedback control to calculate the necessary
 * angular velocity. The command finishes when the robot's heading is within
 * a specified angular threshold.
 * </p>
 */
public class DriveToRotationCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final Supplier<Rotation2d> targetRotationSupplier;
    private final double rotationThreshold; // in radians

    /**
     * Constructs a new DriveToRotationCommand.
     *
     * @param driveSubsystem        the drive subsystem used to rotate the robot
     * @param targetRotationSupplier a supplier of the target heading
     * @param rotationThreshold     the allowable angular error (in radians) before the command finishes
     */
    public DriveToRotationCommand(DriveSubsystem driveSubsystem, Supplier<Rotation2d> targetRotationSupplier, double rotationThreshold) {
        this.driveSubsystem = driveSubsystem;
        this.targetRotationSupplier = targetRotationSupplier;
        this.rotationThreshold = rotationThreshold;
        addRequirements(driveSubsystem);
    }

    /**
     * Called repeatedly while the command is scheduled.
     * <p>
     * Computes the rotational velocity required to rotate toward the current target heading,
     * and commands the drive subsystem accordingly.
     * </p>
     */
    @Override
    public void execute() {
        Rotation2d target = targetRotationSupplier.get();
        double rotationalVelocity = driveSubsystem.getRotationalFeedback(target);
        driveSubsystem.driveRotational(rotationalVelocity);
    }

    /**
     * Returns whether the command has finished.
     * <p>
     * The command finishes when the heading error is below the specified angular threshold.
     * </p>
     *
     * @return {@code true} if the robot is within the desired angular threshold; {@code false} otherwise.
     */
    @Override
    public boolean isFinished() {
        Rotation2d current = driveSubsystem.getRobotPose().getRotation();
        Rotation2d target = targetRotationSupplier.get();
        double error = current.minus(target).getRadians();
        return Math.abs(error) < rotationThreshold;
    }

    /**
     * Called once the command ends or is interrupted.
     * <p>
     * This method stops the robot's rotation by commanding zero angular velocity.
     * </p>
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveRotational(0.0);
    }
}

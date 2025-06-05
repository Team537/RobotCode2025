package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

/**
 * Command to drive the robot to a dynamically supplied translation.
 * <p>
 * This command continuously polls a Translation2d supplier to get the target.
 * The robot uses feedback control to compute the required linear speeds to move
 * toward the target. The command finishes when the translation error is within
 * a defined threshold.
 * </p>
 */
public class DriveToTranslationCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final Supplier<Translation2d> targetTranslationSupplier;
    private final double translationThreshold; // in meters

    /**
     * Constructs a new DriveToTranslationCommand.
     *
     * @param driveSubsystem           the drive subsystem used to move the robot
     * @param targetTranslationSupplier a supplier of the desired target translation
     * @param translationThreshold    the allowable error (in meters) before the command finishes
     */
    public DriveToTranslationCommand(DriveSubsystem driveSubsystem, Supplier<Translation2d> targetTranslationSupplier, double translationThreshold) {
        this.driveSubsystem = driveSubsystem;
        this.targetTranslationSupplier = targetTranslationSupplier;
        this.translationThreshold = translationThreshold;
        addRequirements(driveSubsystem);
    }

    /**
     * Called repeatedly while the command is scheduled.
     * <p>
     * Computes the translational feedback required to drive toward the current
     * target and commands the drive subsystem accordingly.
     * </p>
     */
    @Override
    public void execute() {
        Translation2d target = targetTranslationSupplier.get();
        Translation2d velocity = driveSubsystem.getTranslationalFeedback(target);
        driveSubsystem.driveTranslational(velocity);
    }

    /**
     * Returns whether the command has finished.
     * <p>
     * The command finishes when the translation error (distance) is below the specified threshold.
     * </p>
     *
     * @return {@code true} if the robot is within the desired threshold; {@code false} otherwise
     */
    @Override
    public boolean isFinished() {
        Translation2d current = driveSubsystem.getRobotPose().getTranslation();
        Translation2d target = targetTranslationSupplier.get();
        return current.getDistance(target) < translationThreshold;
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
        driveSubsystem.driveTranslational(Translation2d.kZero);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

/**
 * Drives the robot translationally using velocity input.
 * If the commanded velocity is near zero and the robot is stopped,
 * it holds the robot at its last known position before stopping.
 */
public class DriveWithTranslationalVelocityCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final Supplier<Translation2d> velocitySupplier;

    private Translation2d heldPosition = null;

    public DriveWithTranslationalVelocityCommand(DriveSubsystem driveSubsystem, Supplier<Translation2d> velocitySupplier) {
        this.driveSubsystem = driveSubsystem;
        this.velocitySupplier = velocitySupplier;
    }

    @Override
    public void execute() {
        Translation2d targetVel = velocitySupplier.get();
        boolean zeroTarget = targetVel.getNorm() < 1e-2;

        double vx = driveSubsystem.getVelocity().vxMetersPerSecond;
        double vy = driveSubsystem.getVelocity().vyMetersPerSecond;
        boolean robotStopped = Math.hypot(vx, vy) < 1e-2;

        if (zeroTarget && robotStopped) {
            if (heldPosition == null) {
                // Save the current position the first time we enter "hold" mode
                heldPosition = driveSubsystem.getRobotPose().getTranslation();
            }
            Translation2d feedback = driveSubsystem.getTranslationalFeedback(heldPosition);
            driveSubsystem.driveTranslational(feedback);
        } else {
            // Exit "hold" mode
            heldPosition = null;
            driveSubsystem.driveTranslational(targetVel);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveTranslational(new Translation2d());
        heldPosition = null;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
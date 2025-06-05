package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

/**
 * Drives the robot rotationally using a velocity input.
 * If the commanded angular velocity is near zero and the robot is stopped,
 * it holds the robot at its last known heading before stopping.
 */
public class DriveWithRotationalVelocityCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> velocitySupplier;

    private Rotation2d heldHeading = null;

    public DriveWithRotationalVelocityCommand(DriveSubsystem driveSubsystem, Supplier<Double> velocitySupplier) {
        this.driveSubsystem = driveSubsystem;
        this.velocitySupplier = velocitySupplier;
    }

    @Override
    public void execute() {
        double targetOmega = velocitySupplier.get();
        boolean zeroTarget = Math.abs(targetOmega) < 1e-2;

        double currentOmega = driveSubsystem.getVelocity().omegaRadiansPerSecond;
        boolean robotStopped = Math.abs(currentOmega) < 1e-2;

        if (zeroTarget && robotStopped) {
            if (heldHeading == null) {
                heldHeading = driveSubsystem.getRobotPose().getRotation();
            }
            double feedback = driveSubsystem.getRotationalFeedback(heldHeading);
            driveSubsystem.driveRotational(feedback);
        } else {
            heldHeading = null;
            driveSubsystem.driveRotational(targetOmega);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveRotational(0.0);
        heldHeading = null;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

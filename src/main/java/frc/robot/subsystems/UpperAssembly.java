package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.ScoringHeight;

/**
 * Represents the upper assembly of the robot, which may vary depending on the selected implementation.
 * Provides methods for various assembly-specific actions such as scoring, intaking, climbing, 
 * and manual control. 
 */
public interface UpperAssembly extends Subsystem {

    /**
     * Creates a command to run the coral intake mechanism.
     * This command is used to collect coral.
     *
     * @param robotPoseSupplier A supplier to return the pose of the robot
     * @return A command that controls the coral intake mechanism.
     */
    public Command getCoralIntakeCommand(Supplier<Pose2d> robotPoseSupplier);

    /**
     * Creates a command to score a coral at a specific height.
     * This command operates the mechanism required to place coral at the specified scoring position.
     *
     * @param scoringHeight The target height for scoring the coral.
     * @param robotPoseSupplier A supplier to return the pose of the robot
     * @return A command to control the scoring mechanism at the specified height.
     */
    public Command getCoralScoreCommand(ScoringHeight scoringHeight, Supplier<Pose2d> robotPoseSupplier);

    /**
     * Creates a command to remove algae.
     * This command controls the mechanism for clearing algae.
     *
     * @param robotPoseSupplier A supplier to return the pose of the robot
     * @return A command that handles the removal of algae.
     */
    public Command getRemoveAlgaeCommand(Supplier<Pose2d> robotPoseSupplier);

    /**
     * Creates a command to climb or elevate the robot.
     * This command is used during climbing or traversal operations.
     *
     * @param robotPoseSupplier A supplier to return the pose of the robot
     * @return A command that controls the climbing mechanism.
     */
    public Command getClimbCommand(Supplier<Pose2d> robotPoseSupplier);

    /**
     * Creates a manual control command for the upper assembly.
     * This allows direct control of the upper assembly using an Xbox controller.
     *
     * @param controller The Xbox controller used for manual operation.
     * @return A command for manually controlling the upper assembly.
     */
    public Command getManualCommand(XboxController controller);

    /**
     * Disables all hardware objects associated with the assembly.
     * This method is used to ensure the upper assembly's hardware is safely stopped and no longer active.
     */
    public void disable();

}

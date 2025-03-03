package frc.robot.subsystems.upper_assembly;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.upper_assembly.ScoringHeight;

/**
 * Represents the upper assembly of the robot, which may vary depending on the selected implementation.
 * Provides methods for various assembly-specific actions such as scoring, intaking, climbing, 
 * and manual control. 
 */
public interface UpperAssembly {

    /**
     * Creates a command to run the coral intake mechanism.
     * This command is used to collect coral.
     *
     * @param robotPoseSupplier A supplier to return the pose of the robot
     * @return A command that controls the coral intake mechanism.
     */
    public Command getCoralIntakeCommand();

    /**
     * Creates a command to score a coral at a specific height.
     * This command operates the mechanism required to place coral at the specified scoring position.
     *
     * @param scoringHeight The target height for scoring the coral.
     * @param robotPoseSupplier A supplier to return the pose of the robot
     * @return A command to control the scoring mechanism at the specified height.
     */
    public Command getCoralScoreCommand(ScoringHeight scoringHeight);

    /**
     * Creates a command to remove algae.
     * This command controls the mechanism for clearing algae.
     *
     * @param robotPoseSupplier A supplier to return the pose of the robot
     * @return A command that handles the removal of algae.
     */
    public Command getRemoveAlgaeCommand();

    /**
     * Creates a command to climb or elevate the robot.
     * This command is used during climbing or traversal operations.
     *
     * @param robotPoseSupplier A supplier to return the pose of the robot
     * @return A command that controls the climbing mechanism.
     */
    public Command getClimbCommand();

    /**
     * Creates a manual control command for the upper assembly.
     * This allows direct control of the upper assembly using an Xbox controller.
     *
     * @param controller The Xbox controller used for manual operation.
     * @return A command for manually controlling the upper assembly.
     */
    public Command getManualCommand(XboxController controller);

    /**
     * Sets the supplier that provides a boolean indicating whether the robot is in the scoring position.
     * <p>
     * This supplier should return {@code true} if the robot is correctly positioned for scoring,
     * and {@code false} otherwise.
     * </p>
     *
     * @param supplier a {@link java.util.function.Supplier} that returns a {@code Boolean} value
     *                 representing the robot's scoring position status.
     */
    public void setRobotInScoringPositionSupplier(Supplier<Boolean> supplier);

    /**
     * Sets the supplier that provides a boolean indicating whether the robot is in the intaking position.
     * <p>
     * This supplier should return {@code true} if the robot is correctly positioned for intaking,
     * and {@code false} otherwise.
     * </p>
     *
     * @param supplier a {@link java.util.function.Supplier} that returns a {@code Boolean} value
     *                 representing the robot's intaking position status.
     */
    public void setRobotInIntakingPositionSupplier(Supplier<Boolean> supplier);

    /**
     * Sets the supplier that provides a boolean indicating whether the robot is in the climbing position.
     * <p>
     * This supplier should return {@code true} if the robot is correctly positioned for climbing,
     * and {@code false} otherwise.
     * </p>
     *
     * @param supplier a {@link java.util.function.Supplier} that returns a {@code Boolean} value
     *                 representing the robot's climbing position status.
     */
    public void setRobotInClimbPositionSupplier(Supplier<Boolean> supplier);

}
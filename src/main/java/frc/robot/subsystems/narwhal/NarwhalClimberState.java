package frc.robot.subsystems.narwhal;

/**
 * <h2> NarwhalClimberState </h2>
 * The {@code NarwhalClimberState} enum represents several pre-configured climber positions.
 * <hr>
 * @author Patrick Wang
 * @since v1.2.0
 */
public enum NarwhalClimberState {
    STARTING,
    DEPLOYING,
    CLIMBING,
    CUSTOM, 
    HOLDING, // power is set to zero
}

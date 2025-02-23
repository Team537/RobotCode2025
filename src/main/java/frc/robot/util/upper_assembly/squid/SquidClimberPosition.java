package frc.robot.util.upper_assembly.squid;

import frc.robot.Constants.SquidConstants.SquidClimberConstants;

/**
 * The {@code SquidClimberPosition} enum defines preset positions for the Squid Climber mechanism.
 * <p>
 * Each enum constant corresponds to a position value (in meters) defined in the robot constants.
 * </p>
 */
public enum SquidClimberPosition {
    /** Climber in the down position. */
    DOWN(SquidClimberConstants.DOWN_POSITION),
    /** Climber in the climbed position. */
    CLIMBED(SquidClimberConstants.CLIMBED_POSITION);

    private final double position;

    /**
     * Constructs a new {@code SquidClimberPosition} with the specified position value.
     *
     * @param position The position value in meters.
     */
    SquidClimberPosition(double position) {
        this.position = position;
    }

    /**
     * Gets the position value associated with this climber position.
     *
     * @return The position in meters.
     */
    public double getPosition() {
        return position;
    }
}

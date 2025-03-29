package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.OperatorConstants;

/**
 * <h2> Alliance </h2>
 * An enum value used to keep track of which alliance the robot is currently on. Supports <b>RED</b>, <b>BLUE</b>,
 * and <b>DEMO</b> modes. It should be noted that <b> DEMO </b> will typically not be used during competition and 
 * exists solely to make it easier to write code specifically for certain events.
 * 
 * This enum is primarily used in <code>RobotContainer</code> to configure alliance-specific settings.
 * <hr>
 * @author Cameron Myhre
 * @author Viktor Austad
 * @since v1.0.0
 * @see {@link frc.robot.RobotContainer}
*/
public enum Alliance {
    RED(OperatorConstants.RED_ALLIANCE_OFFSET),
    BLUE(OperatorConstants.BLUE_ALLIANCE_OFFSET),
    DEMO(OperatorConstants.DEMO_ALLIANCE_OFFSET); // Not used during competition! This is only for showcasing our robot code at events!

    private final Rotation2d DRIVER_ROTATIONAL_OFFSET;

    /**
     * Creates a new Alliance enum with the specified driver rotational offset, as a Rotation2d.
     * 
     * @param driverRotationalOffset This enum`s driver rotational offset, as a Rotation2d.
     */
    Alliance(Rotation2d driverRotationalOffset) {
        this.DRIVER_ROTATIONAL_OFFSET = driverRotationalOffset;
    }

    /**
     * Returns the driver rotation offset for each alliance, as a Rotation2d.
     * 
     * @return The driver rotation offset for each alliance, as a Rotation2d.
     */
    public Rotation2d getDriverRotationalOffset() {
        return this.DRIVER_ROTATIONAL_OFFSET;
    }

}

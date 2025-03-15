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

    private Rotation2d driverRotationalOffset;

    Alliance(Rotation2d driverRotationalOffset) {
        this.driverRotationalOffset = driverRotationalOffset;
    }

    public Rotation2d getDriverRotationalOffset() {
        return driverRotationalOffset;
    }

}

package frc.robot.util.autonomous;

/**
 * <h2> StartingPosition </h2>
 * An enum value used to represent which autonomous routine we will be preforming. This is ued inside of {@code} RobotContainer}
 * to determine which commands to schedule during autonomous. Below is the current list of supported autonomous routines:
 * <ul>
 *  <li> <b>LEFT_HIGH_SCORE</b> </li>
 *  <li> <b>RIGHT_HIGH_SCORE</b> </li>
 *  <li> <b>CENTER_HIGH_SCORE</b> </li>
 * </ul>
 * <hr>
 * @author Cameron Myhre
 * @author Viktor Austad
 * @since v1.0.0
 * @see {@link frc.robot.RobotContainer}
 */
public enum StartingPosition {
    LEFT,
    CENTER,
    RIGHT
}

package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * <h2> StartingPosition </h2>
 * An enum value used to represent which starting position the robot will be in when the match starts
 * <ul>
 *  <li> <b>LEFT</b> </li>
 *  <li> <b>RIGHT</b> </li>
 *  <li> <b>CENTER</b> </li>
 * </ul>
 * <hr>
 * @author Cameron Myhre
 * @author Viktor Austad
 * @since v1.0.0
 * @see {@link frc.robot.RobotContainer}
 */
public enum StartingPosition {
    LEFT(),
    CENTER(),
    RIGHT();

    Pose2d pose;

    StartingPosition(Pose2d pose) {
        this.pose = pose;
    }

}

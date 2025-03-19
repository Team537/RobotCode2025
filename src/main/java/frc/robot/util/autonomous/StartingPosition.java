package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.FieldConstants.StartingPoseConstants;

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
    LEFT(StartingPoseConstants.BLUE_LEFT_STARTING_POSE,StartingPoseConstants.RED_LEFT_STARTING_POSE),
    CENTER(StartingPoseConstants.BLUE_CENTER_STARTING_POSE,StartingPoseConstants.RED_CENTER_STARTING_POSE),
    RIGHT(StartingPoseConstants.BLUE_RIGHT_STARTING_POSE,StartingPoseConstants.RED_RIGHT_STARTING_POSE);

    Pose2d bluePose;
    Pose2d redPose;

    StartingPosition(Pose2d bluePose, Pose2d redPose) {
        this.bluePose = bluePose;
        this.redPose = redPose;
    }

    public Pose2d getPose(Alliance alliance) { 
        if (alliance == Alliance.BLUE) {
            return bluePose;
        } else {
            return redPose;
        }
    }

}

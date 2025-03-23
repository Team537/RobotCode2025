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
    LEFT(StartingPoseConstants.BLUE_LEFT_STARTING_POSE, StartingPoseConstants.RED_LEFT_STARTING_POSE),
    CENTER(StartingPoseConstants.BLUE_CENTER_STARTING_POSE, StartingPoseConstants.RED_CENTER_STARTING_POSE),
    RIGHT(StartingPoseConstants.BLUE_RIGHT_STARTING_POSE, StartingPoseConstants.RED_RIGHT_STARTING_POSE);

    private final Pose2d BLUE_POSE;
    private final Pose2d RED_POSE;

    /**
     * Creates a new StartingPosition enum using the given blue and red starting positions, as Pose2ds.
     * 
     * @param bluePose The starting position for the blue alliance, as a Pose2d.
     * @param redPose The starting position for the red alliance, as a Pose2d.
     */
    StartingPosition(Pose2d bluePose, Pose2d redPose) {
        this.BLUE_POSE = bluePose;
        this.RED_POSE = redPose;
    }

    /**
     * Returns teh starting position alongside the associated alliance.
     * 
     * @param alliance The alliance color the position will be taken form.
     * @return The starting position for the specified alliance.
     */
    public Pose2d getPose(Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            return this.BLUE_POSE;
        }

        return this.RED_POSE;
    }
}

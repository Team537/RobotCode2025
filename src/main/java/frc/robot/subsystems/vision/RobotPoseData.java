package frc.robot.subsystems.vision;

/**
 * Simple Data Transfer Object (DTO) for representing the robot's pose
 * and a timestamp in seconds.
 * <hr>
 * @author Cameron Myhre
 * @since v2.0.0
 */
public class RobotPoseData {

    public Pose pose;
    public double timestamp;

    /**
     * Constructs a new RobotPoseData object with the given parameters.
     *
     * @param x           Robot's X position in meters
     * @param y           Robot's Y position in meters
     * @param heading_rad Robot's heading in radians
     * @param timestamp   The current time, e.g., from Timer.getFPGATimestamp()
     */
    public RobotPoseData(double x, double y, double heading_rad, double timestamp) {
        this.pose = new Pose(x, y, heading_rad);
        this.timestamp = timestamp;
    }

    /**
     * Inner class describing the pose portion (x, y, heading_rad).
     */
    public static class Pose {
        public double x;
        public double y;
        public double heading_rad;

        public Pose(double x, double y, double heading_rad) {
            this.x = x;
            this.y = y;
            this.heading_rad = heading_rad;
        }
    }
}

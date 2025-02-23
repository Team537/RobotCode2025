package frc.robot.subsystems.vision;

/**
 * DataPacket encapsulates vision data which may include the robot's pose and frame capture flags,
 * along with a timestamp indicating when the data was recorded.
 * <hr>
 * @author Cameron Myhre
 * @since v2.0.0
 */
public class DataPacket {

    /** The robot's pose (x, y, heading in radians), or {@code null} if not provided. */
    public Pose robotPose;
    
    /** The frame capture information, or {@code null} if not provided. */
    public Capture capture;
    
    /** The timestamp (in seconds or milliseconds depending on context) when the data was recorded. */
    public double timestamp;

    /**
     * Constructs a DataPacket with both the robot pose and frame capture information.
     *
     * @param xMeters             the x-coordinate of the robot in meters
     * @param yMeters             the y-coordinate of the robot in meters
     * @param headingRadians      the heading of the robot in radians
     * @param captureInputFrame   {@code true} to capture the input frame; {@code false} otherwise
     * @param captureOutputFrame  {@code true} to capture the output frame; {@code false} otherwise
     * @param captureDepthFrame   {@code true} to capture the depth frame; {@code false} otherwise
     * @param timestamp           the timestamp when the data was recorded
     */
    public DataPacket(double xMeters, double yMeters, double headingRadians, boolean captureInputFrame,
            boolean captureOutputFrame, boolean captureDepthFrame, double timestamp) {
        this.robotPose = new Pose(xMeters, yMeters, headingRadians);
        this.capture = new Capture(captureInputFrame, captureOutputFrame, captureDepthFrame);
        this.timestamp = timestamp;
    }

    /**
     * Constructs a DataPacket with frame capture information only.
     * The robot pose is not provided and is set to {@code null}.
     *
     * @param captureInputFrame   {@code true} to capture the input frame; {@code false} otherwise
     * @param captureOutputFrame  {@code true} to capture the output frame; {@code false} otherwise
     * @param captureDepthFrame   {@code true} to capture the depth frame; {@code false} otherwise
     * @param timestamp           the timestamp when the data was recorded
     */
    public DataPacket(boolean captureInputFrame, boolean captureOutputFrame, boolean captureDepthFrame,
            double timestamp) {
        this.robotPose = null;
        this.capture = new Capture(captureInputFrame, captureOutputFrame, captureDepthFrame);
        this.timestamp = timestamp;
    }

    /**
     * Constructs a DataPacket with the robot pose only.
     * Frame capture information is not provided and is set to {@code null}.
     *
     * @param xMeters        the x-coordinate of the robot in meters
     * @param yMeters        the y-coordinate of the robot in meters
     * @param headingRadians the heading of the robot in radians
     * @param timestamp      the timestamp when the data was recorded
     */
    public DataPacket(double xMeters, double yMeters, double headingRadians, double timestamp) {
        this.robotPose = new Pose(xMeters, yMeters, headingRadians);
        this.capture = null;
        this.timestamp = timestamp;
    }

    /**
     * Constructs a DataPacket with only a timestamp.
     * Both robot pose and frame capture information are not provided and are set to {@code null}.
     *
     * @param timestamp the timestamp when the data was recorded
     */
    public DataPacket(double timestamp) {
        this.robotPose = null;
        this.capture = null;
        this.timestamp = timestamp;
    }

    /**
     * Inner class describing the pose portion of the data.
     */
    public static class Pose {
        /** The x-coordinate of the robot in meters. */
        public double x;
        
        /** The y-coordinate of the robot in meters. */
        public double y;
        
        /** The heading of the robot in radians. */
        public double heading_rad;

        /**
         * Constructs a Pose with the specified x, y coordinates and heading.
         *
         * @param x            the x-coordinate in meters
         * @param y            the y-coordinate in meters
         * @param heading_rad  the heading in radians
         */
        public Pose(double x, double y, double heading_rad) {
            this.x = x;
            this.y = y;
            this.heading_rad = heading_rad;
        }
    }

    /**
     * Inner class describing the photo capturing portion of the data.
     */
    public static class Capture {
        /** Indicates whether the input frame should be captured. */
        public boolean inputFrame;
        
        /** Indicates whether the output frame should be captured. */
        public boolean outputFrame;
        
        /** Indicates whether the depth frame should be captured. */
        public boolean depthFrame;

        /**
         * Constructs a Capture with the specified frame capture flags.
         *
         * @param inputFrame   {@code true} to capture the input frame; {@code false} otherwise
         * @param outputFrame  {@code true} to capture the output frame; {@code false} otherwise
         * @param depthFrame   {@code true} to capture the depth frame; {@code false} otherwise
         */
        public Capture(boolean inputFrame, boolean outputFrame, boolean depthFrame) {
            this.inputFrame = inputFrame;
            this.outputFrame = outputFrame;
            this.depthFrame = depthFrame;
        }
    }
}

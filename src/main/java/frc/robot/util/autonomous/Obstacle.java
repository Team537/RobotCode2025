package frc.robot.util.autonomous;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * <h2> Obstacle </h2>
 * A datatype class representing a pathfinding obstacle. 
 * It contains a position, as a Translation2d; a radius,as a double; and a timestamp, as a double.  
 * Distances use meters, time uses seconds.
 */
public class Obstacle {

    /**
     * The obstacles translation, with its values in meters.
     */
    private Translation2d obstacleTranslation;

    /**
     * The obstacle`s radius, in meters.
     */
    private double obstacleRadius;            

    /**
     * The obstacle`s timestamp, in seconds.
     */
    private double obstacleTimestamp;

    /**
     * Constructs an Obstacle with the specified translation, radius, and timestamp.
     *
     * @param obstacleTranslation the position of the obstacle in meters.
     * @param obstacleRadius      the radius of the obstacle in meters.
     * @param obstacleTimestamp   the timestamp associated with the obstacle in seconds.
     */
    public Obstacle(Translation2d obstacleTranslation, double obstacleRadius, double obstacleTimestamp) {
        this.obstacleTranslation = obstacleTranslation;
        this.obstacleRadius = obstacleRadius;
        this.obstacleTimestamp = obstacleTimestamp;
    }

    /**
     * Constructs an Obstacle with the specified translation and radius.
     * The timestamp is set to 0.0 seconds.
     *
     * @param obstacleTranslation The position of the obstacle in meters.
     * @param obstacleRadius      The radius of the obstacle in meters.
     */
    public Obstacle(Translation2d obstacleTranslation, double obstacleRadius) {
        this(obstacleTranslation, obstacleRadius, 0.0);
    }

    /**
     * Returns this Obstacle`s translation2d, with its values in meters.
     * 
     * @return This Obstacle`s translation2d, with its values in meters.
     */
    public Translation2d getObstacleTranslation() {
        return obstacleTranslation;
    }

    /**
     * Sets this Obstacle`s translation2d, with its values in meters.
     * 
     * @return The Obstacle`s new translation2d, with its values in meters.
     */
    public void setObstacleTranslation(Translation2d obstacleTranslation) {
        this.obstacleTranslation = obstacleTranslation;
    }

    /**
     * Returns this Obstacle`s radius, in meters.
     * 
     * @return This Obstacle`s radius, in meters.
     */
    public double getObstacleRadius() {
        return obstacleRadius;
    }

    /**
     * Sets this Obstacle`s radius to the specified value, in meters.
     * 
     * @return This Obstacle`s new radius, in meters.
     */
    public void setObstacleRadius(double obstacleRadius) {
        this.obstacleRadius = obstacleRadius;
    }

    /**
     * Returns this Obstacle`s timestamp, in seconds.
     * 
     * @return This Obstacle`s timestamp, in seconds.
     */
    public double getObstacleTimestamp() {
        return obstacleTimestamp;
    }

    /**
     * Sets this Obstacle`s timestamp to the specified value, in seconds.
     * 
     * @return This Obstacle`s new timestamp, in seconds.
     */
    public void setObstacleTimestamp(double obstacleTimestamp) {
        this.obstacleTimestamp = obstacleTimestamp;
    }

    /**
     * Checks if a given point collides with the obstacle.
     * 
     * @param point The point (Translation2d) to check.
     * @return True if the distance between the point and the obstacle's center is less than or equal to the obstacle's radius.
     */
    public boolean collidesWith(Translation2d point) {
        double dx = obstacleTranslation.getX() - point.getX();
        double dy = obstacleTranslation.getY() - point.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        return distance <= obstacleRadius;
    }

    /**
     * Checks if a given circle collides with the obstacle.
     * 
     * @param circleCenter The center of the circle (Translation2d).
     * @param circleRadius The radius of the circle.
     * @return True if the distance between the obstacle's center and the circle's center is less than or equal to the sum of the radii.
     */
    public boolean collidesWith(Translation2d circleCenter, double circleRadius) {
        double dx = obstacleTranslation.getX() - circleCenter.getX();
        double dy = obstacleTranslation.getY() - circleCenter.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        return distance <= (obstacleRadius + circleRadius);
    }
}
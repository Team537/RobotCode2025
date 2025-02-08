package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

public class Obstacle {

    private Translation2d obstacleTranslation; // Meters
    private double obstacleRadius;             // Meters
    private double obstacleTimestamp;          // Seconds

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
     * The timestamp is defaulted to 0.0 seconds.
     *
     * @param obstacleTranslation the position of the obstacle in meters.
     * @param obstacleRadius      the radius of the obstacle in meters.
     */
    public Obstacle(Translation2d obstacleTranslation, double obstacleRadius) {
        this(obstacleTranslation, obstacleRadius, 0.0);
    }

    // Getter and Setter for obstacleTranslation
    public Translation2d getObstacleTranslation() {
        return obstacleTranslation;
    }

    public void setObstacleTranslation(Translation2d obstacleTranslation) {
        this.obstacleTranslation = obstacleTranslation;
    }

    // Getter and Setter for obstacleRadius
    public double getObstacleRadius() {
        return obstacleRadius;
    }

    public void setObstacleRadius(double obstacleRadius) {
        this.obstacleRadius = obstacleRadius;
    }

    // Getter and Setter for obstacleTimestamp
    public double getObstacleTimestamp() {
        return obstacleTimestamp;
    }

    public void setObstacleTimestamp(double obstacleTimestamp) {
        this.obstacleTimestamp = obstacleTimestamp;
    }

    /**
     * Checks if a given point collides with the obstacle.
     * 
     * @param point The point (Translation2d) to check.
     * @return true if the distance between the point and the obstacle's center is less than or equal to the obstacle's radius.
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
     * @return true if the distance between the obstacle's center and the circle's center is less than or equal to the sum of the radii.
     */
    public boolean collidesWith(Translation2d circleCenter, double circleRadius) {
        double dx = obstacleTranslation.getX() - circleCenter.getX();
        double dy = obstacleTranslation.getY() - circleCenter.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        return distance <= (obstacleRadius + circleRadius);
    }
}
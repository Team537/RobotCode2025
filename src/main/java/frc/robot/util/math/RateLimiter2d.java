package frc.robot.util.math;

import edu.wpi.first.math.MathSharedStore;

/**
 * <h2> RateLimiter2d </h2>
 * The {@code RateLimiter} class is used to limit the rate of change of a {@code Vector2d} object.
 * This ensures that the robot controls smoothly and reliably, and is used extensively in {@code DriveSubsystem}.
 * <hr>
 * @author Parker Huibregtse
 * @since v1.1.0
 * @see {@link frc.robot.subsystems.DriveSubsystem}
 */
public class RateLimiter2d {

    // The current value (a 2D vector) being rate-limited
    private Vector2d value;

    // The rate of change (magnitude per second)
    private double rate;

    // The last time the value was adjusted, used for calculating delta time
    private double previousTime;

    // The maximum delta time allowed by the rate limiter
    private double maxDeltaTime;

    /**
     * Creates a 2D rate limiter.
     * 
     * @param initialValue  the initial 2D vector value
     * @param rate          the maximum rate of change (units per second)
     * @param maxDeltaTime  the maximum delta time allowed; use Double.MAX_VALUE for no restriction
     */
    public RateLimiter2d(Vector2d initialValue, double rate, double maxDeltaTime) {
        this.value = initialValue;
        this.rate = rate;
        this.maxDeltaTime = maxDeltaTime;
        this.previousTime = MathSharedStore.getTimestamp(); // Initialize with the current timestamp
    }

    /**
     * Updates the 2D value, limiting the rate of change.
     * 
     * @param targetValue the desired 2D vector value to approach
     * @return the new, rate-limited 2D vector value
     */
    public Vector2d update(Vector2d targetValue) {
        double currentTime = MathSharedStore.getTimestamp();
        double deltaTime = currentTime - previousTime;

        // Enforce maximum delta time
        if (deltaTime > maxDeltaTime) {
            deltaTime = maxDeltaTime;
        }

        // Calculate the maximum distance we can move
        double maxChange = rate * deltaTime;

        // Calculate the vector from the current value to the target value
        Vector2d direction = targetValue.subtract(value);

        // Get the distance to the target value
        double distanceToTarget = direction.magnitude();

        // If the target is within the maxChange distance, go directly to it
        if (distanceToTarget <= maxChange) {
            value = targetValue;
        } else {
            // Otherwise, move in the direction of the target by maxChange
            direction = direction.normalize().scale(maxChange);
            value = value.add(direction);
        }

        // Update the previous time
        previousTime = currentTime;

        return value;
    }

    /**
     * Gets the current 2D vector value of the rate limiter.
     * 
     * @return the current 2D vector value
     */
    public Vector2d getValue() {
        return value;
    }

    /**
     * Sets the value
     * @param value the 2D vector being set to
     */
    public void setValue(Vector2d value) {
        this.value = value;
    }

    /**
     * Sets a new rate for the rate limiter.
     * 
     * @param rate the new rate (units per second)
     */
    public void setRate(double rate) {
        this.rate = rate;
    }

    /**
     * Sets a new maximum delta time for the rate limiter.
     * 
     * @param maxDeltaTime the new maximum delta time
     */
    public void setMaxDeltaTime(double maxDeltaTime) {
        this.maxDeltaTime = maxDeltaTime;
    }
}
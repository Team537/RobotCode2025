package frc.robot.util.math;

import edu.wpi.first.math.MathSharedStore;

/**
 * <h2> RateLimiter </h2>
 * The {@code RateLimiter} class is used to limit the rate of acceleration when driving the robot.
 * This ensures that the robot controls smoothly and reliably, and is used extensively in {@code DriveSubsystem}.
 * <hr>
 * @author Parker Huibregtse
 * @since v1.1.0
 * @see {@link frc.robot.subsystems.DriveSubsystem}
*/
public class RateLimiter {

    // Value whose rate will be limited
    private double value;

    // The rate at which the value will be limited (in units per second)
    private double rate;

    // The last time the value was adjusted, used for calculating delta time
    private double previousTime;

    // The maximum delta time allowed by the rate limiter
    private double maxDeltaTime;

    /**
     * Creates a rate limiter.
     * 
     * @param initialValue the initial value to be set
     * @param rate         the maximum rate of change (units per second)
     * @param maxDeltaTime the maximum delta time allowed; use Double.MAX_VALUE for
     *                     no restriction
     */
    public RateLimiter(double initialValue, double rate, double maxDeltaTime) {
        this.value = initialValue;
        this.rate = rate;
        this.maxDeltaTime = maxDeltaTime;
        this.previousTime = MathSharedStore.getTimestamp(); // Initialize with the current timestamp
    }

    /**
     * Updates the value, limiting the rate of change.
     * 
     * @param targetValue the desired value to approach
     * @return the new, rate-limited value
     */
    public double update(double targetValue) {
        double currentTime = MathSharedStore.getTimestamp();
        double deltaTime = currentTime - previousTime;

        // Enforce maximum delta time
        if (deltaTime > maxDeltaTime) {
            deltaTime = maxDeltaTime;
        }

        // Calculate the maximum change allowed
        double maxChange = rate * deltaTime;

        // Limit the change to the target value
        if (targetValue > value) {
            value = Math.min(value + maxChange, targetValue);
        } else if (targetValue < value) {
            value = Math.max(value - maxChange, targetValue);
        }

        // Update the previous time
        previousTime = currentTime;

        return value;
    }

    /**
     * Gets the current value of the rate limiter.
     * 
     * @return the current value
     */
    public double getValue() {
        return value;
    }

    /**
     * Sets the value to a double
     * @param value a double to set the value to
     */
    public void setValue(double value) {
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
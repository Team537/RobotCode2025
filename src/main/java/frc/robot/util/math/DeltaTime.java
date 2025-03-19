package frc.robot.util.math;

import edu.wpi.first.math.MathSharedStore;

/**
 * A simple class to calculate the elapsed time (delta time) between successive calls,
 * using MathSharedStore.getTimestamp() to obtain the current time in seconds.
 * <hr>
 * @author Parker Huibregtse
 * @version 2.0.0
 */
public class DeltaTime {

    // The time of the previous update in seconds.
    private double lastTime;

    /**
     * Constructor: Initializes the timer with the current timestamp.
     */
    public DeltaTime() {
        lastTime = MathSharedStore.getTimestamp();
    }

    /**
     * Returns the elapsed time in seconds since the last call to getDeltaTime(),
     * and resets the timer to the current timestamp.
     *
     * @return The elapsed time in seconds.
     */
    public double getDeltaTime() {
        double currentTime = MathSharedStore.getTimestamp();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        return deltaTime;
    }

    /**
     * Returns the elapsed time in seconds since the last reset without resetting the timer.
     *
     * @return The elapsed time in seconds since the last update.
     */
    public double peekDeltaTime() {
        double currentTime = MathSharedStore.getTimestamp();
        return currentTime - lastTime;
    }

    /**
     * Resets the timer to the current timestamp.
     */
    public void reset() {
        lastTime = MathSharedStore.getTimestamp();
    }
}

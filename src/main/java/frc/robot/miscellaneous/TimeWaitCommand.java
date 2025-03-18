package frc.robot.miscellaneous;

import edu.wpi.first.wpilibj2.command.Command;

public class TimeWaitCommand extends Command {

    // Storage for the wait time in nanoseconds.
    private final long waitTimeNanoseconds;
    private long completionTimeNanoseconds;

    /**
     * Creates a new TimeWaitCommand.
     *
     * @param timeMilliseconds The time the command will wait for, in milliseconds. Must be non-negative.
     * @throws IllegalArgumentException if timeMilliseconds is negative.
     */
    public TimeWaitCommand(double timeMilliseconds) {
        if (timeMilliseconds < 0) {
            throw new IllegalArgumentException("timeMilliseconds must be non-negative");
        }
        // Convert milliseconds to nanoseconds accurately.
        this.waitTimeNanoseconds = Math.round(timeMilliseconds * 1000000L);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        completionTimeNanoseconds = System.nanoTime() + waitTimeNanoseconds;
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        // No periodic action is necessary.
    }

    /**
     * Returns true when the wait time has elapsed.
     */
    @Override
    public boolean isFinished() {
        return System.nanoTime() >= completionTimeNanoseconds;
    }
}

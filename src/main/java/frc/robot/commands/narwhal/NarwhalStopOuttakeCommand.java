package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalIntakeOuttake;

/**
 * Instant command to stop the outtake mechanism.
 * <p>
 * When scheduled, this command immediately calls <code>intakeOuttake.stop()</code>
 * to halt the outtake. This command finishes instantly.
 * </p>
 */
public class NarwhalStopOuttakeCommand extends Command {

    private final NarwhalIntakeOuttake intakeOuttake;

    /**
     * Constructor for NarwhalStopOuttakeCommand.
     *
     * @param intakeOuttake The subsystem that controls the intake and outtake.
     */
    public NarwhalStopOuttakeCommand(NarwhalIntakeOuttake intakeOuttake) {
        this.intakeOuttake = intakeOuttake;
        // Declare subsystem dependency to prevent other commands from using it simultaneously.
        addRequirements(intakeOuttake);
    }

    /**
     * Called just before this Command runs for the first time.
     * Immediately stops the outtake mechanism.
     */
    @Override
    public void initialize() {
        intakeOuttake.stop();
    }

    /**
     * Since this command is instant, it is finished immediately.
     *
     * @return true to indicate the command has completed.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}

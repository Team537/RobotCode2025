package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalIntakeOuttake;

/**
 * Command that continuously activates the outtake mechanism.
 * <p>
 * This command calls <code>intakeOuttake.outtake()</code> to run the outtake.
 * It finishes when <code>intakeOuttake.isCoralSensorTriggered()</code> returns true,
 * indicating that coral is in the chute.
 * <strong>Important:</strong> The outtake mechanism is not stopped when this command ends,
 * allowing it to continue running until stopped by another command.
 * </p>
 */
public class NarwhalIntakeCommand extends Command {

    private final NarwhalIntakeOuttake intakeOuttake;

    /**
     * Constructor for NarwhalOuttakeCommand.
     *
     * @param intakeOuttake The subsystem that controls the intake and outtake.
     */
    public NarwhalIntakeCommand(NarwhalIntakeOuttake intakeOuttake) {
        this.intakeOuttake = intakeOuttake;
        // Declare subsystem dependency to prevent other commands from using it simultaneously.
        addRequirements(intakeOuttake);
    }

    @Override
    public void initialize() {
        intakeOuttake.intake();
    }

    @Override
    public void execute() {
        intakeOuttake.intake();
    }

    @Override
    public boolean isFinished() {
        return intakeOuttake.isCoralSensorTriggered();
    }

}

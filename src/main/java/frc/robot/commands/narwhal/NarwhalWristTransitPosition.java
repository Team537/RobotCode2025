package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalWrist;

/**
 * Command to move the elevator and wrist to their transit positions.
 * This is used during robot motion between scoring positions.
 */
public class NarwhalWristTransitPosition extends Command {
    
    // Subsystem references for the elevator and wrist
    private final NarwhalWrist wrist;

    /**
     * Create a new NarwhalTransitPositionCommand with the given elevator and wrist mechanisms.
     * 
     * @param elevator The robot`s elevator mechanism.
     * @param wrist The robot`s wrist mechanism.
     */
    public NarwhalWristTransitPosition(NarwhalWrist wrist) {
        this.wrist = wrist;

        // Declare subsystem dependencies to prevent conflicting commands.
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.goToTransitAngle();
    }

    @Override
    public boolean isFinished() {
        return wrist.isAtTargetPosition(); //
    }
}
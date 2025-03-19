package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalWrist;

/**
 * Command to move the elevator and wrist to their transit positions.
 * This is used during robot motion between scoring positions.
 */
public class NarwhalTransitPositionCommand extends Command {
    
    // Subsystem references for the elevator and wrist
    private final NarwhalElevator elevator;
    private final NarwhalWrist wrist;

    public NarwhalTransitPositionCommand(NarwhalElevator elevator, NarwhalWrist wrist) {
        this.elevator = elevator;
        this.wrist = wrist;

        // Declare subsystem dependencies to prevent conflicting commands.
        addRequirements(elevator, wrist);
    }

    @Override
    public void execute() {
        elevator.goToMinimumHeight();
        wrist.goToTransitAngle();
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtTargetPosition() && wrist.isAtTargetPosition();
    }
}
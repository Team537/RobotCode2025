package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalWrist;
import frc.robot.util.upper_assembly.ScoringHeight;

/**
 * Command to move the elevator and wrist to the scoring positions corresponding to a given scoring height.
 * This command sets the elevator to the desired height and the wrist to the appropriate outtake angle,
 * and then finishes once both subsystems have reached their target positions.
 */
public class NarwhalScorePositionCommand extends Command {
    
    // Subsystem references for elevator and wrist
    private final NarwhalElevator elevator;
    private final NarwhalWrist wrist;
    
    // The desired scoring height (e.g., L1, L2, L3, or L4)
    private final ScoringHeight scoringHeight;
    
    public NarwhalScorePositionCommand(NarwhalElevator elevator, NarwhalWrist wrist, ScoringHeight scoringHeight) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.scoringHeight = scoringHeight;
        
        // Declare subsystem dependencies to avoid conflicts with other commands.
        addRequirements(elevator, wrist);
    }
    
    @Override
    public void execute() {
        // Command the elevator to move to the target scoring height.
        elevator.goToScoreHeight(scoringHeight);
        
        // Command the wrist to move to the outtake angle for the specified scoring height.
        wrist.goToScoreAngle(scoringHeight);
    }
    
    @Override
    public boolean isFinished() {
        return elevator.isAtTargetPosition() && wrist.isAtTargetPosition();
    }
}
package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalWrist;


/** 
 * A command that moves the manipulator down to descore algae
 */
public class NarwhalAlgaeRemoveCommand extends Command{
    private final NarwhalElevator narwhalElevator;
    private final NarwhalWrist narwhalWrist;
    private final boolean isTopRow;

    /**
     * Creates a new NarwhalAlgaeRemoveCommand.
     * @param narwhalElevator The NarwhalElevator subsystem to use.
     * @param narwhalWrist The NarwhalWrist subsystem to use.
     * @param isTopRow Whether the algae is in the top row or not
     */
    public NarwhalAlgaeRemoveCommand(NarwhalElevator narwhalElevator, NarwhalWrist narwhalWrist, boolean isTopRow){
        this.narwhalElevator = narwhalElevator;
        this.narwhalWrist = narwhalWrist;
        this.isTopRow = isTopRow;
        addRequirements(narwhalElevator, narwhalWrist);        
    }

    @Override
    public void execute(){
        // Move the elevator to the algae descore position and press down on the algae to descore it
        narwhalElevator.goToAlgaeDescorePosition(isTopRow, true);
        // Move the wrist to the algae angle
        narwhalWrist.goToAlgaeAngle();
    }

    @Override
    public boolean isFinished(){
        // Finish the command if the Coral sensor is no longer triggered (i.e. the Coral has been scored)
        return narwhalElevator.isAtTargetPosition() && narwhalWrist.isAtTargetPosition();
    }
}

package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalWrist;


/** 
 * A command that climbs using the Narwhal upper assembly.
 * Will lower the elevator to the minimum height, move the wrist out of the way to the
 * climb angle, and then climb. If the robot is not ready to climb, the climber arm will be moved to the deploy position.
 */
public class NarwhalAlgaeDescoreCommand extends Command{
    private final NarwhalElevator narwhalElevator;
    private final NarwhalWrist narwhalWrist;

    /**
     * Creates a new NarwhalAlgaeDescoreCommand.
     * @param narwhalElevator The NarwhalElevator subsystem to use.
     * @param narwhalWrist The NarwhalWrist subsystem to use.
     */
    public NarwhalAlgaeDescoreCommand(NarwhalElevator narwhalElevator, NarwhalWrist narwhalWrist){
        this.narwhalElevator = narwhalElevator;
        this.narwhalWrist = narwhalWrist;
        addRequirements(narwhalElevator, narwhalWrist);        
    }

    @Override
    public void execute(){
        narwhalElevator.goToMinimumHeight();
        narwhalWrist.goToClimbAngle();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

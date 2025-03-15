package frc.robot.commands.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalClimber;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalWrist;
import frc.robot.util.upper_assembly.narwhal.NarwhalClimberState;


/** 
 * A command that climbs using the Narwhal upper assembly.
 * Will lower the elevator to the minimum height, move the wrist out of the way to the
 * climb angle, and then climb. If the robot is not ready to climb, the climber arm will be moved to the deploy position.
 */
public class NarwhalClimbCommand extends Command{
    private final NarwhalElevator narwhalElevator;
    private final NarwhalWrist narwhalWrist;
    private final NarwhalClimber narwhalClimber;
    private final Supplier<Boolean> readyToClimbSupplier;

    /**
     * Creates a new NarwhalClimbCommand.
     * @param narwhalElevator The NarwhalElevator subsystem to use.
     * @param narwhalWrist The NarwhalWrist subsystem to use.
     * @param narwhalClimber The NarwhalClimber subsystem to use.
     * @param readyToClimbSupplier A supplier that returns true when the robot is ready to climb.
     */
    public NarwhalClimbCommand(NarwhalElevator narwhalElevator, NarwhalWrist narwhalWrist, NarwhalClimber narwhalClimber, Supplier<Boolean> readyToClimbSupplier){
        this.narwhalElevator = narwhalElevator;
        this.narwhalWrist = narwhalWrist;
        this.narwhalClimber = narwhalClimber;
        this.readyToClimbSupplier = readyToClimbSupplier;
        addRequirements(narwhalElevator, narwhalWrist, narwhalClimber);        
    }

    @Override
    public void execute(){
        narwhalElevator.goToMinimumHeight();
        narwhalWrist.goToClimbAngle();

        // If the robot is ready to climb, climb, otherwise move the climber arm to the deploy position
        if(readyToClimbSupplier.get() && narwhalClimber.isAtDeployAngle() && narwhalWrist.isAtTargetPosition() && narwhalElevator.isAtTargetPosition()){
            narwhalClimber.climb();
        }
        else if (narwhalClimber.currentState != NarwhalClimberState.CLIMBING && !narwhalClimber.isAtDeployAngle()){
            narwhalClimber.goToDeploy();
        }
    }

    @Override
    public boolean isFinished(){
        // Finish the command if the Coral sensor is no longer triggered (i.e. the Coral has been scored)
        return narwhalClimber.isAtClimbAngle();
    }
}

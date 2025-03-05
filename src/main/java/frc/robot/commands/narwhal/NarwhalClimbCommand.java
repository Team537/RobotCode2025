package frc.robot.commands.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalClimber;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalIntakeOuttake;
import frc.robot.subsystems.narwhal.NarwhalWrist;


/** 
 * A command that scores the Coral using the Narwhal upper assembly. (NOTE: IF THERE IS NO CORAL OR THE CORAL SENSOR IS MALFUNCTIONING, THIS COMMAND WILL END PREMATURLY)
 * Will raise the elevator to the appropriate height, move the wrist to the
 * appropriate angle, wait for the ready to score supplier to return true, and then score the Coral.
 */
public class NarwhalClimbCommand extends Command{
    private final NarwhalElevator narwhalElevator;
    private final NarwhalWrist narwhalWrist;
    private final NarwhalClimber narwhalClimber;
    private final Supplier<Boolean> readyToClimbSupplier;

    /**
     * Creates a new NarwhalScoreCommand.
     * @param narwhalElevator The NarwhalElevator subsystem to use.
     * @param narwhalWrist The NarwhalWrist subsystem to use.
     * @param narwhalIntakeOuttake The NarwhalIntakeOuttake subsystem to use.
     * @param readyToScoreSupplier A supplier that returns true when the robot is ready to score.
     * @param targetScoringHeight The height to score the Coral at.
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
        if(readyToClimbSupplier.get()){
            narwhalClimber.climb();
        }
        else {
            narwhalClimber.goToDeploy();
        }
    }

    @Override
    public boolean isFinished(){
        // Finish the command if the Coral sensor is no longer triggered (i.e. the Coral has been scored)
        return narwhalClimber.isAtClimbAngle();
    }
}

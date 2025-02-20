package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalClimber;
import frc.robot.util.NarwhalClimberState;

public class ManualNarwhalClimberCommand extends Command {
    
    private final NarwhalClimber narwhalClimber;
    private final XboxController controller;
    
    /**
     * Creates a manual squid climber command that toggles between down and climbed positions.
     * @param squidClimber The climber subsystem to control.
     * @param controller The Xbox controller used to drive the climber.
     */
    public ManualNarwhalClimberCommand(NarwhalClimber narwhalClimber, XboxController controller) {
        this.narwhalClimber = narwhalClimber;
        this.controller = controller;
        addRequirements(narwhalClimber);
    }
    
    @Override
    public void execute() {
        if(controller.getStartButtonPressed()){
            if (narwhalClimber.currentState != NarwhalClimberState.DEPLOYING){
                narwhalClimber.goToDeploy();
            }
            else if(narwhalClimber.currentState != NarwhalClimberState.CLIMBING){
                narwhalClimber.climb();
            }
        }
    }

}

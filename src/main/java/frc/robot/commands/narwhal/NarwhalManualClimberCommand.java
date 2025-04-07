package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalClimber;

public class NarwhalManualClimberCommand extends Command {
    
    private final NarwhalClimber narwhalClimber;
    private final XboxController controller;
    
    /**
     * Creates a manual squid climber command that toggles between down and climbed positions.
     * 
     * @param narwhalClimber The climber subsystem to control.
     * @param controller The Xbox controller used to drive the climber.
     */
    public NarwhalManualClimberCommand(NarwhalClimber narwhalClimber, XboxController controller) {
        this.narwhalClimber = narwhalClimber;
        this.controller = controller;
        addRequirements(narwhalClimber);
    }
    
    @Override
    public void execute() {
        // deploy
        if (controller.getPOV() == 180){
            narwhalClimber.goToDeploy();
        }
        // climb
        else if(controller.getPOV() == 0){
            narwhalClimber.climb();
        }

        else if(controller.getPOV() == 90){
            narwhalClimber.climbManualDown(); // will rotate climb out
        }

        else if(controller.getPOV() == 270){
            narwhalClimber.climbManualUp(); // will rotate climb in
        }
    }

}

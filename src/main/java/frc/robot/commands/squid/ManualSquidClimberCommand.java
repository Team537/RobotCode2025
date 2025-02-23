package frc.robot.commands.squid;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.squid.SquidClimber;
import frc.robot.util.upper_assembly.squid.SquidClimberPosition;

public class ManualSquidClimberCommand extends Command {
    
    private final SquidClimber squidClimber;
    private final XboxController controller;
    
    /**
     * Track the current climber state: false means DOWN, true means CLIMBED.
     */
    private boolean isClimbed = false;
    
    /**
     * For edge-detection of the Start button.
     */
    private boolean previousStartButtonState = false;
    
    /**
     * Creates a manual squid climber command that toggles between down and climbed positions.
     * 
     * @param squidClimber The climber subsystem to control.
     * @param controller The Xbox controller used to drive the climber.
     */
    public ManualSquidClimberCommand(SquidClimber squidClimber, XboxController controller) {
        this.squidClimber = squidClimber;
        this.controller = controller;
        addRequirements(squidClimber);
    }
    
    @Override
    public void execute() {

        // Read the current state of the Start button.
        boolean currentStartButtonState = controller.getStartButton();
        
        // Edge detection: only toggle when the button is pressed (rising edge).
        if (currentStartButtonState && !previousStartButtonState) {

            // Toggle the climber state.
            isClimbed = !isClimbed;
            
            // Set the position based on the new state.
            if (isClimbed) {

                // Move the climber to the climbed position.
                squidClimber.setPosition(SquidClimberPosition.CLIMBED);
            } else {

                // Move the climber to the down position.
                squidClimber.setPosition(SquidClimberPosition.DOWN);
            }
        }
        
        // Update the previous state of the Start button for edge detection.
        previousStartButtonState = currentStartButtonState;
    }

}

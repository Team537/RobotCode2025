package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.util.upper_assembly.ScoringHeight;

public class NarwhalManualElevatorCommand extends Command {
    private final NarwhalElevator narwhalElevator;
    private final XboxController xboxController;

    /**
     * Creates a manual narwhal elevator command that moves the elevator to different positions based on button presses.
     * 
     * @param narwhalElevator The elevator subsystem to control.
     * @param controller The Xbox controller used to drive the elevator.
     */
    public NarwhalManualElevatorCommand(NarwhalElevator narwhalElevator, XboxController controller) {
        this.narwhalElevator = narwhalElevator;
        this.xboxController = controller;
        //addRequirements(narwhalElevator);
    }

    @Override
    public void execute() {
        // when the intake command is called & the wrist has moved into position for intaking, go to intake angle.
        if(xboxController.getRightBumperButton()) {
            narwhalElevator.goToIntakeHeight();
        }
        // Back button = move to L1 height
        else if(xboxController.getBackButton()){
            narwhalElevator.goToScoreHeight(ScoringHeight.L1);
        }
        // A button = move to L2 height
        else if(xboxController.getAButton()){
            narwhalElevator.goToScoreHeight(ScoringHeight.L2);
        }
        // B button = move to L3 height
        else if(xboxController.getBButton()){
            narwhalElevator.goToScoreHeight(ScoringHeight.L3);
        }
        // Y button = move to L4 height
        else if(xboxController.getYButton()){
            narwhalElevator.goToScoreHeight(ScoringHeight.L4);
        }
    }
}

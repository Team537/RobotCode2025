package frc.robot.commands.squid;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.squid.SquidManipulator;
import frc.robot.util.upper_assembly.ScoringHeight;

public class ManualSquidManipulatorCommand extends Command {

    SquidManipulator squidManipulator;
    XboxController controller;
    
    boolean outtakeActive = false;
    boolean outtakeInputLastState = false;
    boolean coralSensedLastState = false;

    /**
     * creates a manual squid manipulator command for an xbox controller
     * @param squidManipulator the squid manipulator to run
     * @param controller the xbox controller to drive the subsystem
     */
    public ManualSquidManipulatorCommand(SquidManipulator squidManipulator, XboxController controller) {

        this.squidManipulator = squidManipulator;
        this.controller = controller;
        addRequirements(squidManipulator);

    }

    @Override
    public void execute() {

        //Deactivate rollers on sensor resing edge
        boolean coralSensed = squidManipulator.senseCoral();
        if (coralSensed && !coralSensedLastState) {
            outtakeActive = false;
        }
        coralSensedLastState = coralSensed;

        //Reverse and deactivate on right bumper press
        if (controller.getRightBumperButton()) {
            outtakeActive = false;
            squidManipulator.reverseFullSpeed();
        } else {
            
            //X button toggles intake
            boolean currentInput = controller.getXButton();
            if (currentInput && !outtakeInputLastState) {
                outtakeActive = !outtakeActive;
            }
            outtakeInputLastState = currentInput;

            if (outtakeActive) {
                squidManipulator.outtakeAngled();
            } else {
                squidManipulator.holdManipulator();
            }
            
        }

        //Set scoring height
        if (controller.getBackButton()) {
            squidManipulator.setScoringHeight(ScoringHeight.L1);
        }
        if (controller.getAButton()) {
            squidManipulator.setScoringHeight(ScoringHeight.L2);
        }
        if (controller.getBButton()) {
            squidManipulator.setScoringHeight(ScoringHeight.L3);
        }
        if (controller.getYButton()) {
            squidManipulator.setScoringHeight(ScoringHeight.L4);
        }

    }
    
}

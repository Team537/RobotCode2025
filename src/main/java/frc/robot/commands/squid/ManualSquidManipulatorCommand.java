package frc.robot.commands.squid;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.squid.SquidManipulator;

public class ManualSquidManipulatorCommand extends Command {

    SquidManipulator squidManipulator;
    XboxController controller;

    boolean intaking = false;

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

    /**
     * sets whether the robot is in its intaking mode or not
     * @param intaking
     */
    public void setIntaking(boolean intaking) {
        this.intaking = intaking;
    }

    @Override
    public void execute() {
        if (controller.getXButton() == controller.getRightBumperButton()) {
            //if both buttons are or aren't pressed, stop the intake
            squidManipulator.holdManipulator();
        } else {
            if (controller.getXButton()) {
                if (intaking) { 
                    squidManipulator.outtakeFullSpeed(); //intake without a differential
                } else {
                    squidManipulator.outtakeAngled(squidManipulator.getScoringPosition()); //outtake with a differential
                }
            } else {
                squidManipulator.reverseFullSpeed(); //reversing to unjam coral
            }
        }
    }
    
}

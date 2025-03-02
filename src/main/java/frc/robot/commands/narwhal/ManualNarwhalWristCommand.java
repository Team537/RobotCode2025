package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalWrist;

public class ManualNarwhalWristCommand extends Command {
    private final XboxController xBoxController;
    private final NarwhalWrist narwhalWrist;

    /**
     * Creates a manual narwhal climber command that controls the climber subsystem.
     * @param squidClimber The climber subsystem to control.
     * @param controller The Xbox controller used to control the subsystem.
     */
    public ManualNarwhalWristCommand(XboxController xboxController, NarwhalWrist narwhalWrist){
        this.xBoxController = xboxController;
        this.narwhalWrist = narwhalWrist;
        addRequirements(narwhalWrist);
    }

    @Override
    public void execute(){
        // Right bumper = Intake
        if(xBoxController.getRightBumperButton()){
            narwhalWrist.goToIntakeAngle();
        }
        // Left bumper = Algae
        else if(xBoxController.getLeftBumperButtonPressed()){
            narwhalWrist.goToAlgaeAngle();
        }
        // Back button = Score L1 Wrist Angle
        else if(xBoxController.getBackButton()){
            narwhalWrist.goToL1WristAngle();
        } 
        // A button = Score L2 Wrist Angle
        else if(xBoxController.getAButton()){
            narwhalWrist.goToL2WristAngle();
        } 
        // B button = Score L3 Wrist Angle
        else if(xBoxController.getBButton()){
            narwhalWrist.goToL3WristAngle();
        } 
        // Y button = Score L4 Wrist Angle
        else if(xBoxController.getYButton()){
            narwhalWrist.goToL4WristAngle();
        }
    }
}
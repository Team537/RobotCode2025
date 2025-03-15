package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalWrist;
import frc.robot.util.upper_assembly.ScoringHeight;

public class NarwhalManualWristCommand extends Command {
    private final XboxController xBoxController;
    private final NarwhalWrist narwhalWrist;

    /**
     * Creates a manual narwhal climber command that controls the climber subsystem.
     * @param squidClimber The climber subsystem to control.
     * @param controller The Xbox controller used to control the subsystem.
     */
    public NarwhalManualWristCommand(NarwhalWrist narwhalWrist, XboxController xboxController){
        this.xBoxController = xboxController;
        this.narwhalWrist = narwhalWrist;
        //addRequirements(narwhalWrist);
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
            narwhalWrist.goToScoreAngle(ScoringHeight.L1);
        } 
        // A button = Score L2 Wrist Angle
        else if(xBoxController.getAButton()){
            narwhalWrist.goToScoreAngle(ScoringHeight.L2);
        } 
        // B button = Score L3 Wrist Angle
        else if(xBoxController.getBButton()){
            narwhalWrist.goToScoreAngle(ScoringHeight.L3);
        } 
        // Y button = Score L4 Wrist Angle
        else if(xBoxController.getYButton()){
            narwhalWrist.goToScoreAngle(ScoringHeight.L4);
        }
        // Climb button = Move to wrist out of the way
        else if(xBoxController.getStartButton()){
            narwhalWrist.goToClimbAngle();
        }
    }
}
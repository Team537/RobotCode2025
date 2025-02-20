package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalIntakeOuttake;

public class NarwhalManualIntakeOuttakeCommand extends Command {
    private final NarwhalIntakeOuttake narwhalIntakeOuttake;
    private final XboxController xboxController;

    public NarwhalManualIntakeOuttakeCommand(NarwhalIntakeOuttake narwhalIntakeOuttake, XboxController xboxController){
        this.narwhalIntakeOuttake = narwhalIntakeOuttake;
        this.xboxController = xboxController;
        addRequirements(narwhalIntakeOuttake);
    }

    @Override
    public void execute(){
        // right bumper (intake) = intake
        if(xboxController.getRightBumperButton()){
            narwhalIntakeOuttake.intake();
        }
        // x button (outtake) = outtake
        else if (xboxController.getXButton()){
            narwhalIntakeOuttake.outtake();
        }
        // if not intaking or outtaking = hold piece
        else {
            narwhalIntakeOuttake.hold();
        }
    }
}

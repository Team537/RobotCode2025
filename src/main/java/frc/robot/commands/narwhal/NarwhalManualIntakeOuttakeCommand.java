package frc.robot.commands.narwhal;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalIntakeOuttake;

public class NarwhalManualIntakeOuttakeCommand extends Command {
    private final NarwhalIntakeOuttake narwhalIntakeOuttake;
    private final XboxController xboxController;

    /**
     * Creates a new instance of the NarwhalManualIntakeOuttakeCommand, which allows for manual control of the 
     * Narwhal's intake/outtake during teleop.
     * 
     * @param narwhalIntakeOuttake The {@link NarwhalIntakeOuttake} subsystem used by the robot.
     * @param xboxController The Xbox controller used to control the intake.
     */
    public NarwhalManualIntakeOuttakeCommand(NarwhalIntakeOuttake narwhalIntakeOuttake, XboxController xboxController){
        this.narwhalIntakeOuttake = narwhalIntakeOuttake;
        this.xboxController = xboxController;
        //addRequirements(narwhalIntakeOuttake);
    }

    @Override
    public void execute() {
        // Right bumper (intake) = intake
        if (xboxController.getRightBumperButton()) {
            narwhalIntakeOuttake.intake();
        }

        // X button (outtake) = outtake
        else if (xboxController.getXButton()) {
            narwhalIntakeOuttake.outtake();
        }

        // If not intaking or outtaking = hold piece
        else {
            narwhalIntakeOuttake.hold();
        }
    }
}

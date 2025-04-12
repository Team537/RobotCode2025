package frc.robot.commands.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalIntakeOuttake;
import frc.robot.util.NarwhalElevatorState;

public class NarwhalManualIntakeOuttakeCommand extends Command {
    private final NarwhalIntakeOuttake narwhalIntakeOuttake;
    private final XboxController xboxController;
    private final Supplier<NarwhalElevatorState> narwhalElevatorStateSupplier;

    /**
     * Creates a new instance of the NarwhalManualIntakeOuttakeCommand, which allows for manual control of the 
     * Narwhal's intake/outtake during teleop.
     * 
     * @param narwhalIntakeOuttake The {@link NarwhalIntakeOuttake} subsystem used by the robot.
     * @param xboxController The Xbox controller used to control the intake.
     */
    public NarwhalManualIntakeOuttakeCommand(NarwhalIntakeOuttake narwhalIntakeOuttake, XboxController xboxController, Supplier<NarwhalElevatorState> narwhalElevatorStateSupplier) {
        this.narwhalIntakeOuttake = narwhalIntakeOuttake;
        this.xboxController = xboxController;
        this.narwhalElevatorStateSupplier = narwhalElevatorStateSupplier;
        //addRequirements(narwhalIntakeOuttake);
    }

    @Override
    public void execute() {
        // X button (outtake) = outtake
        if (xboxController.getXButton()) {
            // During L1, do slow outtake
            if (narwhalElevatorStateSupplier.get() == NarwhalElevatorState.L1) {
                narwhalIntakeOuttake.slowOuttake();
            } 
            // Default to normal outtake for all other states
            else {
                narwhalIntakeOuttake.outtake();
            }
        }

        // Right bumper (intake) = intake
        else if (xboxController.getRightBumperButton()) {
            narwhalIntakeOuttake.intake();
        }

        // If not intaking or outtaking = hold piece
        else {
            narwhalIntakeOuttake.hold();
        }
    }
}

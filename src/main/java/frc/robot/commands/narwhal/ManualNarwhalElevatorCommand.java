package frc.robot.commands.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.narwhal.NarwhalElevator;

public class ManualNarwhalElevatorCommand extends Command {
    private final NarwhalElevator narwhalElevator;
    private final XboxController xboxController;
    private final Supplier<Boolean> narwhalWristReadyToIntakeSupplier;

    /**
     * Creates a manual narwhal elevator command that moves the elevator to different positions based on button presses.
     * @param narwhalElevator The elevator subsystem to control.
     * @param controller The Xbox controller used to drive the elevator.
     * @param narwhalWristReadyToIntakeSupplier A supplier that indicates if the narwhal wrist is ready to intake.
     */
    public ManualNarwhalElevatorCommand(
            NarwhalElevator narwhalElevator, 
            XboxController controller, 
            Supplier<Boolean> narwhalWristReadyToIntakeSupplier) {
        this.narwhalElevator = narwhalElevator;
        this.xboxController = controller;
        this.narwhalWristReadyToIntakeSupplier = narwhalWristReadyToIntakeSupplier;
        addRequirements(narwhalElevator);
    }

    @Override
    public void execute() {
        // when the intake command is called & the wrist has moved into position for intaking, go to intake angle.
        if(xboxController.getRightBumperButtonPressed() && narwhalWristReadyToIntakeSupplier.get()) {
            narwhalElevator.goToIntakeHeight();
        }
        // Back button = move to L1 height
        else if(xboxController.getBackButton()){
            narwhalElevator.goToScoreHeightL1();
        }
        // A button = move to L2 height
        else if(xboxController.getAButton()){
            narwhalElevator.goToScoreHeightL2();
        }
        // B button = move to L3 height
        else if(xboxController.getBButton()){
            narwhalElevator.goToScoreHeightL3();
        }
        // Y button = move to L4 height
        else if(xboxController.getYButton()){
            narwhalElevator.goToScoreHeightL4();
        }
    }
}
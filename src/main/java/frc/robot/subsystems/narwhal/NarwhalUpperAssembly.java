package frc.robot.subsystems.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.narwhal.NarwhalManualClimberCommand;
import frc.robot.commands.narwhal.NarwhalManualElevatorCommand;
import frc.robot.commands.narwhal.NarwhalManualIntakeOuttakeCommand;
import frc.robot.commands.narwhal.NarwhalManualWristCommand;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;
import frc.robot.util.upper_assembly.ScoringHeight;

public class NarwhalUpperAssembly extends UpperAssemblyBase {
    private final NarwhalIntakeOuttake narwhalIntakeOuttake;
    private final NarwhalWrist narwhalWrist;
    private final NarwhalElevator narwhalElevator;
    private final NarwhalClimber narwhalClimber;

    public NarwhalUpperAssembly(){
        super();
        narwhalIntakeOuttake = new NarwhalIntakeOuttake();
        narwhalWrist = new NarwhalWrist();
        narwhalElevator = new NarwhalElevator();
        narwhalClimber = new NarwhalClimber();
    }

    public Command getCoralIntakeCommand(Supplier<Pose2d> robotPoseSupplier) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }
    

    public Command getCoralIntakeCommand() {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getCoralScoreCommand(ScoringHeight scoringHeight) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getRemoveAlgaeCommand() {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getClimbCommand() {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getManualCommand(XboxController controller) {
        NarwhalManualIntakeOuttakeCommand narwhalManualIntakeOuttakeCommand = new NarwhalManualIntakeOuttakeCommand(narwhalIntakeOuttake, controller);
        NarwhalManualWristCommand narwhalManualWristCommand = new NarwhalManualWristCommand(narwhalWrist, controller);
        NarwhalManualElevatorCommand narwhalManualElevatorCommand = new NarwhalManualElevatorCommand(narwhalElevator, controller, narwhalWrist::readyToIntake);
        NarwhalManualClimberCommand narwhalManualClimberCommand = new NarwhalManualClimberCommand(narwhalClimber, controller);

        ParallelCommandGroup manualParallelCommandGroup = new ParallelCommandGroup(
            narwhalManualIntakeOuttakeCommand,
            narwhalManualWristCommand,
            narwhalManualElevatorCommand,
            narwhalManualClimberCommand
        );
        manualParallelCommandGroup.addRequirements(this);
        return manualParallelCommandGroup;
    }

    public void disable() {}

}

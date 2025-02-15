package frc.robot.subsystems.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.UpperAssembly;
import frc.robot.util.ScoringHeight;

public class NarwhalUpperAssembly implements UpperAssembly {
    NarwhalWrist narwhalWrist = new NarwhalWrist();
    NarwhalIntakeOuttake narwhalIntakeOuttake = new NarwhalIntakeOuttake();
    NarwhalElevator narwhalElevator = new NarwhalElevator();
    
    public Command getCoralIntakeCommand(Supplier<Pose2d> robotPoseSupplier) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getCoralScoreCommand(ScoringHeight scoringHeight, Supplier<Pose2d> robotPoseSupplier) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getRemoveAlgaeCommand(Supplier<Pose2d> robotPoseSupplier) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getClimbCommand(Supplier<Pose2d> robotPoseSupplier) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getManualCommand(XboxController controller) {
        ParallelCommandGroup manualCommand = new ParallelCommandGroup(
            new RunCommand(() -> {
                    narwhalWrist.runXBoxController(controller);
                },
                narwhalWrist
            ),
            new RunCommand(() -> {
                    narwhalIntakeOuttake.runXBoxController(controller);
                },
                narwhalIntakeOuttake
            ),
            new RunCommand(() -> {
                    narwhalElevator.runXBoxController(controller);
                },
                narwhalElevator
            )
        );
        manualCommand.addRequirements(this);
        return manualCommand;
    }

    public void disable() {}

}

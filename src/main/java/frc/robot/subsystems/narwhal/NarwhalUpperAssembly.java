package frc.robot.subsystems.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;
import frc.robot.util.upper_assembly.ScoringHeight;

public class NarwhalUpperAssembly extends UpperAssemblyBase {
    
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
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public void disable() {}

}

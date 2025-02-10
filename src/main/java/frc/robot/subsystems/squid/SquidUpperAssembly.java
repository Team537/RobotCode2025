package frc.robot.subsystems.squid;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.UpperAssembly;
import frc.robot.util.ScoringHeight;

public class SquidUpperAssembly implements UpperAssembly {
    
    public Command getCoralIntakeCommand(Supplier<Pose2d> robotPoseSupplier) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/}
        );
    }

    public Command getCoralScoreCommand(ScoringHeight scoringHeight, Supplier<Pose2d> robotPoseSupplier) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/}
        );
    }

    public Command getRemoveAlgaeCommand(Supplier<Pose2d> robotPoseSupplier) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/}
        );
    }

    public Command getClimbCommand(Supplier<Pose2d> robotPoseSupplier) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/}
        );
    }

    public Command getManualCommand(XboxController controller) {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/}
        );
    }

    public void disable() {}

}

package frc.robot.subsystems.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;
import frc.robot.util.upper_assembly.ScoringHeight;

public class NarwhalUpperAssembly extends UpperAssemblyBase {

    public NarwhalUpperAssembly() {
        super();
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
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public void disable() {}

}

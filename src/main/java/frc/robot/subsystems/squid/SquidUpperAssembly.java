package frc.robot.subsystems.squid;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.squid.ManualSquidManipulatorCommand;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;
import frc.robot.util.field.AlgaeRemovalPosition;
import frc.robot.util.upper_assembly.ScoringHeight;

public class SquidUpperAssembly extends UpperAssemblyBase {

    SquidManipulator squidManipulator;
    
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

    public Command getRemoveAlgaeCommand(AlgaeRemovalPosition algaeRemovalPosition) {
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

    public Command getLowerCommand() {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getManualCommand(XboxController controller) {
        if (squidManipulator == null) {
            squidManipulator = new SquidManipulator();
        }
        Command manualCommand = new ParallelCommandGroup(
            new ManualSquidManipulatorCommand(squidManipulator,controller)
        );
        manualCommand.addRequirements(this);
        return manualCommand;
    }

}

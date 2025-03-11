package frc.robot.subsystems.narwhal;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.narwhal.NarwhalClimbCommand;
import frc.robot.commands.narwhal.NarwhalCoralIntakeCommand;
import frc.robot.commands.narwhal.NarwhalCoralScoreCommand;
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


    public Command getCoralIntakeCommand() {
        NarwhalCoralIntakeCommand narwhalCoralIntakeCommand = new NarwhalCoralIntakeCommand(
            narwhalElevator, 
            narwhalWrist, 
            narwhalIntakeOuttake, 
            robotInClimbPositionSupplier
        );
        narwhalCoralIntakeCommand.addRequirements(this);
        return narwhalCoralIntakeCommand;
    }

    public Command getCoralScoreCommand(ScoringHeight scoringHeight) {
        NarwhalCoralScoreCommand narwhalCoralScoreCommand = new NarwhalCoralScoreCommand(
            narwhalElevator, 
            narwhalWrist, 
            narwhalIntakeOuttake, 
            robotInClimbPositionSupplier,
            scoringHeight
        );
        narwhalCoralScoreCommand.addRequirements(this);
        return narwhalCoralScoreCommand;
    }

    public Command getRemoveAlgaeCommand() {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getClimbCommand() {
        NarwhalClimbCommand narwhalClimbCommand = new NarwhalClimbCommand(
            narwhalElevator, 
            narwhalWrist, 
            narwhalClimber, 
            robotInClimbPositionSupplier
        );
        narwhalClimbCommand.addRequirements(this);
        return narwhalClimbCommand;
    }

    public Command getManualCommand(XboxController controller) {
        NarwhalManualIntakeOuttakeCommand narwhalManualIntakeOuttakeCommand = new NarwhalManualIntakeOuttakeCommand(narwhalIntakeOuttake, narwhalElevator, controller);
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

    public void disable() {

    }

}

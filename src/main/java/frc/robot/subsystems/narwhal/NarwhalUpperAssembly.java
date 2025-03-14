package frc.robot.subsystems.narwhal;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.NarwhalConstants;
import frc.robot.Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants;
import frc.robot.commands.narwhal.NarwhalClimbCommand;
import frc.robot.commands.narwhal.NarwhalCoralIntakeCommand;
import frc.robot.commands.narwhal.NarwhalCoralScoreCommand;
import frc.robot.commands.narwhal.NarwhalManualClimberCommand;
import frc.robot.commands.narwhal.NarwhalManualElevatorCommand;
import frc.robot.commands.narwhal.NarwhalManualIntakeOuttakeCommand;
import frc.robot.commands.narwhal.NarwhalManualWristCommand;
import frc.robot.commands.narwhal.NarwhalOuttakeCommand;
import frc.robot.commands.narwhal.NarwhalScorePositionCommand;
import frc.robot.commands.narwhal.NarwhalStopOuttakeCommand;
import frc.robot.commands.narwhal.NarwhalTransitPositionCommand;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;
import frc.robot.util.upper_assembly.ScoringHeight;

public class NarwhalUpperAssembly extends UpperAssemblyBase {

    private Supplier<Boolean> canRaiseLiftSupplier = ()->{return true;};
    private final NarwhalIntakeOuttake intakeOuttake;
    private final NarwhalWrist wrist;
    private final NarwhalElevator elevator;
    private final NarwhalClimber climber;

    public NarwhalUpperAssembly(){
        super();
        intakeOuttake = new NarwhalIntakeOuttake();
        wrist = new NarwhalWrist();
        elevator = new NarwhalElevator();
        climber = new NarwhalClimber();
    }

    public void setCanRaiseLiftSupplier(Supplier<Boolean> supplier) {
        canRaiseLiftSupplier = supplier;
    }


    public Command getCoralIntakeCommand() {
        NarwhalCoralIntakeCommand narwhalCoralIntakeCommand = new NarwhalCoralIntakeCommand(
            elevator, 
            wrist, 
            intakeOuttake, 
            robotInClimbPositionSupplier
        );
        narwhalCoralIntakeCommand.addRequirements(this);
        return narwhalCoralIntakeCommand;
    }

    public Command getCoralScoreCommand(ScoringHeight scoringHeight) {

        Command command = 
            (
                (
                    new NarwhalTransitPositionCommand(elevator,wrist)
                    .until(canRaiseLiftSupplier::get)
                ).andThen(
                    new NarwhalScorePositionCommand(elevator,wrist,scoringHeight)
                ).andThen(
                    new WaitUntilCommand(robotInScoringPositionSupplier::get)
                ).andThen(
                    new NarwhalOuttakeCommand(intakeOuttake)
                ).andThen(
                    new WaitCommand(NarwhalIntakeOuttakeConstants.CORAL_INTAKE_DETECTION_DELAY)
                ).andThen(
                    new NarwhalStopOuttakeCommand(intakeOuttake)
                )
            ).handleInterrupt(intakeOuttake::hold);   
        command.addRequirements(this);
        return command;

        /*NarwhalCoralScoreCommand narwhalCoralScoreCommand = new NarwhalCoralScoreCommand(
            narwhalElevator, 
            narwhalWrist, 
            narwhalIntakeOuttake, 
            robotInClimbPositionSupplier,
            scoringHeight
        );
        narwhalCoralScoreCommand.addRequirements(this);
        return narwhalCoralScoreCommand;*/

    }

    public Command getRemoveAlgaeCommand() {
        return new RunCommand(
            () -> {/*PLACEHOLDER, DO NOT USE RUN COMMANDS!*/},
            this
        );
    }

    public Command getClimbCommand() {
        NarwhalClimbCommand narwhalClimbCommand = new NarwhalClimbCommand(
            elevator, 
            wrist, 
            climber, 
            robotInClimbPositionSupplier
        );
        narwhalClimbCommand.addRequirements(this);
        return narwhalClimbCommand;
    }

    public Command getManualCommand(XboxController controller) {
        NarwhalManualIntakeOuttakeCommand narwhalManualIntakeOuttakeCommand = new NarwhalManualIntakeOuttakeCommand(intakeOuttake, controller);
        NarwhalManualWristCommand narwhalManualWristCommand = new NarwhalManualWristCommand(wrist, controller);
        NarwhalManualElevatorCommand narwhalManualElevatorCommand = new NarwhalManualElevatorCommand(elevator, controller, wrist::readyToIntake);
        NarwhalManualClimberCommand narwhalManualClimberCommand = new NarwhalManualClimberCommand(climber, controller);

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

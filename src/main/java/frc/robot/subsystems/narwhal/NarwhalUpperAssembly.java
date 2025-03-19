package frc.robot.subsystems.narwhal;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.NarwhalConstants;
import frc.robot.Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants;
import frc.robot.commands.narwhal.NarwhalAlgaePreparePositionCommand;
import frc.robot.commands.narwhal.NarwhalAlgaeRemoveCommand;
import frc.robot.commands.narwhal.NarwhalClimbCommand;
import frc.robot.commands.narwhal.NarwhalCoralIntakeCommand;
import frc.robot.commands.narwhal.NarwhalCoralScoreCommand;
import frc.robot.commands.narwhal.NarwhalIntakeCommand;
import frc.robot.commands.narwhal.NarwhalIntakePositionCommand;
import frc.robot.commands.narwhal.NarwhalManualClimberCommand;
import frc.robot.commands.narwhal.NarwhalManualElevatorCommand;
import frc.robot.commands.narwhal.NarwhalManualIntakeOuttakeCommand;
import frc.robot.commands.narwhal.NarwhalManualWristCommand;
import frc.robot.commands.narwhal.NarwhalOuttakeCommand;
import frc.robot.commands.narwhal.NarwhalScorePositionCommand;
import frc.robot.commands.narwhal.NarwhalStopOuttakeCommand;
import frc.robot.commands.narwhal.NarwhalTransitPositionCommand;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;
import frc.robot.util.field.AlgaeRemovalPosition;
import frc.robot.util.upper_assembly.ScoringHeight;

public class NarwhalUpperAssembly extends UpperAssemblyBase {

    private Supplier<Boolean> canRaiseLiftSupplier = ()->{return true;};
    
    private boolean drivetrainCanRemoveAlgae = false;

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
        Command coralIntakeCommand = (
            (
                new WaitUntilCommand(canRaiseLiftSupplier::get)
                .deadlineFor(new NarwhalTransitPositionCommand(elevator,wrist))
            ).andThen(
                new NarwhalIntakePositionCommand(elevator, wrist)
            ).andThen(
                new WaitUntilCommand(robotInIntakingPositionSupplier::get)
            ).andThen(
                new NarwhalIntakeCommand(intakeOuttake)
            )
        );
        coralIntakeCommand.addRequirements(this);
        return coralIntakeCommand;
    }

    public Command getCoralScoreCommand(ScoringHeight scoringHeight) {

        Command command = 
            (
                (
                    new WaitUntilCommand(canRaiseLiftSupplier::get)
                    .deadlineFor(new NarwhalTransitPositionCommand(elevator,wrist))
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

    }

    public Command getRemoveAlgaeCommand(AlgaeRemovalPosition algaeRemovalPosition) {
        /*
        Will keep the upper assembly in transit position until the robot is ready to descore the algae.
        Then it will move the manipulator into position to descore the algae and wait until it can descore the algae
        Then it will move down to descore the algae and set the descored algae flag to true
         */
        Command command = 
            (
                (
                    new WaitUntilCommand(canRaiseLiftSupplier::get)
                    .deadlineFor(new NarwhalTransitPositionCommand(elevator,wrist))
                )
                .andThen(
                    new NarwhalAlgaePreparePositionCommand(elevator, wrist, algaeRemovalPosition.isTopRow())
                )
                .andThen(
                    new WaitUntilCommand(robotInAlgaeRemovingPositionSupplier::get)
                )
                .andThen(
                    new NarwhalAlgaeRemoveCommand(elevator, wrist, algaeRemovalPosition.isTopRow())
                    .alongWith(
                        new InstantCommand(()->drivetrainCanRemoveAlgae = true) // Set the flag to true when the algae is descored
                    )
                )
            ).handleInterrupt(
                ()->{
                    drivetrainCanRemoveAlgae = false;
                    wrist.goToTransitAngle();
                }
            );
            
        command.addRequirements(this);

        // returns the remove algae command
        return command;
    }

    public Command getLowerCommand() {
        return new NarwhalTransitPositionCommand(elevator, wrist);
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

}

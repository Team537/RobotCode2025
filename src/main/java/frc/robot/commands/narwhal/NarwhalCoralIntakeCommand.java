package frc.robot.commands.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Configs.Narwhal;
import frc.robot.Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalIntakeOuttake;
import frc.robot.subsystems.narwhal.NarwhalWrist;


/** 
 * A command that scores the Coral using the Narwhal upper assembly. (NOTE: IF CORAL IS STUCK OR THE CORAL SENSOR IS MALFUNCTIONING, THIS COMMAND WILL NOT END)
 * Will raise the elevator to the appropriate height, move the wrist to the
 * appropriate angle, wait for the ready to intake supplier to return true, and then intake the Coral.
 */
public class NarwhalCoralIntakeCommand extends Command{
    private final NarwhalElevator narwhalElevator;
    private final NarwhalWrist narwhalWrist;
    private final NarwhalIntakeOuttake narwhalIntakeOuttake;
    private final Supplier<Boolean> readyToIntakeSupplier;

    /** Keeps track of how long the command has been running to allow the command to autonomously stop if it's running for too long*/
    private final Timer intakeDelayTimer;

    /**
     * Creates a new NarwhalScoreCommand.
     * @param narwhalElevator The NarwhalElevator subsystem to use.
     * @param narwhalWrist The NarwhalWrist subsystem to use.
     * @param narwhalIntakeOuttake The NarwhalIntakeOuttake subsystem to use.
     * @param readyToIntakeSupplier A supplier that returns true when the robot is ready to score.
     */
    public NarwhalCoralIntakeCommand(NarwhalElevator narwhalElevator, NarwhalWrist narwhalWrist, NarwhalIntakeOuttake narwhalIntakeOuttake, Supplier<Boolean> readyToIntakeSupplier){
        this.narwhalElevator = narwhalElevator;
        this.narwhalWrist = narwhalWrist;
        this.narwhalIntakeOuttake = narwhalIntakeOuttake;
        this.readyToIntakeSupplier = readyToIntakeSupplier;
        addRequirements(narwhalElevator, narwhalWrist, narwhalIntakeOuttake);

        intakeDelayTimer = new Timer();
    }

    @Override
    public void initialize(){
        // Reset the intake delay timer when the command starts
        intakeDelayTimer.stop();
        intakeDelayTimer.reset();
    }

    @Override
    public void execute(){
        // Move the elevator and wrist to the appropriate positions
        narwhalElevator.goToIntakeHeight();
        narwhalWrist.goToIntakeAngle();

        // If the robot is ready to intake, intake the Coral, otherwise do nothing with the intake/outtake
        if(readyToIntakeSupplier.get()){
            narwhalIntakeOuttake.intake();
        }

        // Start the intake delay timer if the Coral sensor is triggered and the intake delay timer isn't already running
        if(narwhalIntakeOuttake.isCoralSensorTriggered()){
            if (!intakeDelayTimer.isRunning()){
                intakeDelayTimer.start();
            }
        }
    }

    @Override
    public boolean isFinished(){
        // Finish the command if the Coral sensor is triggered (i.e. the Coral has been intaked) and the intake delay timer has elapsed (prevents premature ending)
        return narwhalIntakeOuttake.isCoralSensorTriggered() && intakeDelayTimer.hasElapsed(NarwhalIntakeOuttakeConstants.CORAL_INTAKE_DETECTION_DELAY);
    }

    @Override
    public void end(boolean interrupted){
        // Stop the intake/outtake motor when the command ends
        narwhalIntakeOuttake.hold();
        intakeDelayTimer.stop();
    }
}

package frc.robot.commands.narwhal;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.NarwhalConstants.NarwhalIntakeOuttakeConstants;
import frc.robot.subsystems.narwhal.NarwhalElevator;
import frc.robot.subsystems.narwhal.NarwhalIntakeOuttake;
import frc.robot.subsystems.narwhal.NarwhalWrist;
import frc.robot.util.upper_assembly.ScoringHeight;


/** 
 * A command that scores the Coral using the Narwhal upper assembly. (NOTE: IF THERE IS NO CORAL OR THE CORAL SENSOR IS MALFUNCTIONING, THIS COMMAND WILL END PREMATURLY)
 * Will raise the elevator to the appropriate height, move the wrist to the
 * appropriate angle, wait for the ready to score supplier to return true, and then score the Coral.
 */
public class NarwhalCoralScoreCommand extends Command{
    private final NarwhalElevator narwhalElevator;
    private final NarwhalWrist narwhalWrist;
    private final NarwhalIntakeOuttake narwhalIntakeOuttake;
    private final Supplier<Boolean> readyToScoreSupplier;

    private ScoringHeight targetScoringHeight;

    private final Timer outtakeDelayTimer;

    /**
     * Creates a new NarwhalScoreCommand.
     * @param narwhalElevator The NarwhalElevator subsystem to use.
     * @param narwhalWrist The NarwhalWrist subsystem to use.
     * @param narwhalIntakeOuttake The NarwhalIntakeOuttake subsystem to use.
     * @param readyToScoreSupplier A supplier that returns true when the robot is ready to score.
     * @param targetScoringHeight The height to score the Coral at.
     */
    public NarwhalCoralScoreCommand(NarwhalElevator narwhalElevator, NarwhalWrist narwhalWrist, NarwhalIntakeOuttake narwhalIntakeOuttake, Supplier<Boolean> readyToScoreSupplier, ScoringHeight targetScoringHeight){
        this.narwhalElevator = narwhalElevator;
        this.narwhalWrist = narwhalWrist;
        this.narwhalIntakeOuttake = narwhalIntakeOuttake;
        this.readyToScoreSupplier = readyToScoreSupplier;
        addRequirements(narwhalElevator, narwhalWrist, narwhalIntakeOuttake);

        outtakeDelayTimer = new Timer();
    }

    public ScoringHeight getTargetScoringHeight(){
        return targetScoringHeight;
    }

    @Override
    public void initialize(){
        // Reset the outtake delay timer when the command starts
        outtakeDelayTimer.stop();
        outtakeDelayTimer.reset();
    }

    @Override
    public void execute(){
        // Move the elevator and wrist to the appropriate positions
        switch (targetScoringHeight){
            case L1:
                narwhalElevator.goToScoreHeightL1();
                narwhalWrist.goToL1WristAngle();
                break;
            case L2:
                narwhalElevator.goToScoreHeightL2();
                narwhalWrist.goToL2WristAngle();
                break;
            case L3:
                narwhalElevator.goToScoreHeightL3();
                narwhalWrist.goToL3WristAngle();
                break;
            case L4:
                narwhalElevator.goToScoreHeightL4();
                narwhalWrist.goToL4WristAngle();
                break;
        }
        
        // If the robot is ready to score, score the Coral, otherwise do nothing with the intake/outtake
        if(readyToScoreSupplier.get()  && narwhalElevator.isAtTargetPosition() && narwhalWrist.isAtTargetPosition()){
            narwhalIntakeOuttake.outtake();
        }

        // Start the outtake delay timer if the Coral sensor is no longer triggered and the outtake delay timer isn't already running
        if(!narwhalIntakeOuttake.isCoralSensorTriggered()){
            if (!outtakeDelayTimer.isRunning()){
                outtakeDelayTimer.start();
            }
        }
    }

    @Override
    public boolean isFinished(){
        // Finish the command if the Coral sensor is no longer triggered (i.e. the Coral has been scored) and the outtake delay timer has elapsed (prevents premature ending)
        return !narwhalIntakeOuttake.isCoralSensorTriggered() && outtakeDelayTimer.hasElapsed(NarwhalIntakeOuttakeConstants.CORAL_OUTTAKE_DETECTION_DELAY);
    }

    @Override
    public void end(boolean interrupted){
        // Stop the intake/outtake motor when the command ends
        narwhalIntakeOuttake.stop();

        // Stop & reset the outtake delay timer when the command ends
        outtakeDelayTimer.stop();
        outtakeDelayTimer.reset();
    }
}

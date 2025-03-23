package frc.robot.subsystems.upper_assembly;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.field.AlgaeRemovalPosition;
import frc.robot.util.upper_assembly.ScoringHeight;

/**
 * This class is intended to act as a "no upper assembly attached" instance.
 * 
 * Each command just waits two seconds.
 */
public class StubUpperAssembly extends UpperAssemblyBase {
    public Command getCoralIntakeCommand() {
        System.out.println("[STUB ASSEMBLY] Coral Intake");
        return new WaitCommand(2);
    }

    public Command getCoralScoreCommand(ScoringHeight scoringHeight) {
        System.out.println("[STUB ASSEMBLY] Coral Score");
        return new WaitCommand(2);
    }

    public Command getRemoveAlgaeCommand(AlgaeRemovalPosition algaeRemovalPosition) {
        System.out.println("[STUB ASSEMBLY] Remove Algae");
        return new WaitCommand(2);
    }

    public Command getLowerCommand() {
        System.out.println("[STUB ASSEMBLY] Lower");
        return new WaitCommand(2);
    }

    public Command getClimbCommand() {
        System.out.println("[STUB ASSEMBLY] Climb");
        return new WaitCommand(2);
    }

    public Command getManualCommand(XboxController controller) {
        System.out.println("[STUB ASSEMBLY] Manual");
        return new InstantCommand(() -> {}, this);
    }
}

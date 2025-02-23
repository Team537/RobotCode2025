package frc.robot.subsystems.upper_assembly;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class UpperAssemblyBase extends SubsystemBase implements UpperAssembly {
    
    @Override
    public void disable() {
        System.err.println("Error: Upper Assembly does not provide functionality to disable hardware in the event of an emergency.");
    }
}

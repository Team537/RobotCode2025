package frc.robot.util.upper_assembly;

import frc.robot.subsystems.narwhal.NarwhalUpperAssembly;
import frc.robot.subsystems.squid.SquidUpperAssembly;
import frc.robot.subsystems.upper_assembly.StubUpperAssembly;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;

/**
 * A factory class tha constructs the upper assembly. This is used to fuel our hot-swappable code capable of 
 * supporting two different upper assemblies.
 */
public class UpperAssemblyFactory {
    public static UpperAssemblyBase createUpperAssembly(UpperAssemblyType upperAssemblyType) { 
        switch (upperAssemblyType) {
            case SQUID:
                return new SquidUpperAssembly();
            case NARWHAL:
                return new NarwhalUpperAssembly();
            default:
                return new StubUpperAssembly();
        }
    }
}

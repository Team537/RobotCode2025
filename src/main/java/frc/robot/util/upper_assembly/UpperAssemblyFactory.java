package frc.robot.util.upper_assembly;

import frc.robot.subsystems.narwhal.NarwhalUpperAssembly;
import frc.robot.subsystems.squid.SquidUpperAssembly;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;

public class UpperAssemblyFactory {
    
    public static UpperAssemblyBase createUpperAssembly(UpperAssemblyType upperAssemblyType) {
        
        switch(upperAssemblyType) {
            case SQUID:
                return new SquidUpperAssembly();
            case NARWHAL:
                return new NarwhalUpperAssembly();
            default:
                return null;
        }

    }

}

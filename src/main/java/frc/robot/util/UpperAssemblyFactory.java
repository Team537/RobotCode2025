package frc.robot.util;

import frc.robot.subsystems.UpperAssembly;
import frc.robot.subsystems.narwhal.NarwhalUpperAssembly;
import frc.robot.subsystems.squid.SquidUpperAssembly;

public class UpperAssemblyFactory {
    
    public static UpperAssembly createUpperAssembly(UpperAssemblyType upperAssemblyType) {
        
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

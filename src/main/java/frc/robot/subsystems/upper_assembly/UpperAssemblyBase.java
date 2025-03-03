package frc.robot.subsystems.upper_assembly;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * <h2> UpperAssemblyBase </h2>
 * An abstract class used to serve as an intermediary class between the {@link UpperAssembly} interface and the {@link SubsystemBase}
 * abstract class. This class allows us to use SubsystemBase functionality on the upper assemblies without having the {@link SubsystemBase}
 * interface extend the {@link SubsystemBase} class, preventing unexpected behavior due to an interface having pre-defined, functional code
 * contained within methods.
 * <hr>
 * @author Cameron Myhre
 * @since 2.0.0
 */
public abstract class UpperAssemblyBase extends SubsystemBase implements UpperAssembly {
    
    protected Supplier<Boolean> robotInScoringPositionSupplier;
    protected Supplier<Boolean> robotInIntakingPositionSupplier;
    protected Supplier<Boolean> robotInClimbPositionSupplier;

    protected UpperAssemblyBase() {
        robotInScoringPositionSupplier = () -> {return true;};
        robotInIntakingPositionSupplier = () -> {return true;};
        robotInClimbPositionSupplier = () -> {return true;};
    }

    public void setRobotInScoringPositionSupplier(Supplier<Boolean> supplier) {
        robotInScoringPositionSupplier = supplier;
    }

    public void setRobotInIntakingPositionSupplier(Supplier<Boolean> supplier) {
        robotInIntakingPositionSupplier = supplier;
    }

    public void setRobotInClimbPositionSupplier(Supplier<Boolean> supplier) {
        robotInClimbPositionSupplier = supplier;
    }

}

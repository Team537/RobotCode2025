package frc.robot.subsystems.narwhal;

/**
 * <h2> NarwhalIntakeOuttakeState </h2>
 * The {@code NarwhalIntakeOuttakeState} enum is an enum value used to represent the current state of
 * the intake and outtake motor. This simplifies code, and makes controlling the motors far easier.
 * <hr>
 * @author Patrick Wang
 * @since v1.2.0
 */
public enum NarwhalIntakeOuttakeState {
    INTAKING,
    OUTTAKING,
    STOPPED,
    ACTIVE_HOLDING,
    CUSTOM, // set to a custom intake-outtake percentage
}
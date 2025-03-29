package frc.robot.util;

/**
 * <h2> NarwhalWristState </h2>
 * The {@code NarwhalWristState} enum represents several pre-configured wrist positions.
 * <hr>
 * @author Patrick Wang
 * @since v1.2.0
 */
public enum NarwhalWristState {
    /** In position for intaking from human player station*/
    INTAKING, 

    /** In position to descore algae from coral*/
    ALGAE, 
    
    /** In position to score L1 coral */
    L1, 
    
    /** In position to score L2 coral */
    L2, 
    
    /** In position to score L3 coral */
    L3, 
    
    /** In position to score L4 coral :) */
    L4, 

    /** In position to climb */
    CLIMB,

    /** In position to move with a coral */
    TRANSIT,
    
    /** Narwhal Wrist power is set to zero */
    STOPPED, 
    
    /** Custom target position */
    CUSTOM, 
}

package frc.robot.util.LEDs;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

/**
 * <h2> LEDConfiguration </h2>
 * An enum value used to represent several preset LED patterns, allowing for easy switching between complex patterns.
 * <hr>
 * @author Cameron Myhre
 * @author Viktor Austad
 * @since v1.0.0
 * @see {@link frc.robot.RobotContainer}
 */
public enum LEDConfiguration {
    MANUEL_DRIVE (LEDPattern.solid(new Color(199, 102, 255)).breathe(Seconds.of(0.25))),
    PROGRAM_STOP (LEDPattern.solid(new Color(237, 28, 35))),
    MANUEL_VICTORY_DANCE(LEDPattern.rainbow(0, 255).blink(Seconds.of(0.45))),
    AUTO_CORAL_SCORE(LEDPattern.gradient(GradientType.kContinuous, new Color(255, 79, 120), new Color(51, 177, 255), new Color(255, 129, 51))), // #ff8133 #33b1ff #ff4f78
    AUTO_ALGAE_REMOVAL(LEDPattern.gradient(GradientType.kDiscontinuous, new Color(181, 223, 166), new Color(71, 109, 59))), // #b5dfa6 #476d3b
    AUTO_SCORING_FAIL(LEDPattern.solid(new Color(131, 8, 9)).blink(Seconds.of(0.5))), // #830809
    AUTO_CORAL_GRAB(LEDPattern.solid(new Color(247, 183, 192)).breathe(Seconds.of(1))); // #f7b7c0

    private final LEDPattern LED_PATTERN;

    /**
     * Creates a new LED configuration with the given LED pattern.
     * 
     * @param ledPattern The LED Pattern associated with each LED configuration.
     */
    LEDConfiguration(LEDPattern ledPattern) {
        this.LED_PATTERN = ledPattern;
    }

    /**
     * Returns this LEDConfiguration's LED patterns.
     * 
     * @return This LEDConfiguration's LED patterns.
     */
    public LEDPattern getLEDPattern() {
        return this.LED_PATTERN;
    }
}
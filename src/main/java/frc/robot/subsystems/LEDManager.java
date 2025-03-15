package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.util.LEDs.LEDConfiguration;

/**
 * <h2> LEDManager </h2>
 * The LEDManager class is a class that allows for LEDs to be easily controlled and configured to 
 * several preset configurations. This simplifies communicating information to the driver, as
 * complex actions can be communicated through unique LED Patterns. Below is a breakdown of all
 * these patterns:
 * <ul>
 *  <li> <strong> Purple Pulsing: </strong> Manual drive active </li>
 *  <li> <strong> Solid Red: </strong> Program stopped  </li>
 *  <li> <strong> Rainbow Blinking: </strong> Victory dance  </li>
 *  <li> <strong> Orange-Blue-Magenta Gradient: </strong> Autonomously scoring coral  </li>
 *  <li> <strong> Light-Dark Green Gradient: </strong> Autonomously scoring algae </li>
 *  <li> <strong> Dark Red Blinking: </strong> Autoscoring failure  </li>
 *  <li> <strong> Light Pink Breathing: </strong> Autonomously grabbing coral  </li>
 * </ul>
 * <hr>
 * @author Kinjal Bhardwa
 * @author Cameron Myhre
 * @since v1.2.0
 */
public class LEDManager {

    // Hardware
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    /**
     * Creates a new LEDManager object using the provided PWM port number.
     * 
     * @param portNum The port number the LED strip is plugged into on the PWM.
     */
    public LEDManager(int portNum) {

        // Create new AddressableLED and AddressableLEDBuffer objects.
        this.ledStrip = new AddressableLED(portNum);
        this.ledBuffer = new AddressableLEDBuffer(60); // TODO: Figure out actual number. 60 is just a placeholder.
        
        // Setup the LED strip hardware
        this.ledStrip.setLength(this.ledBuffer.getLength());
        this.ledStrip.setData(ledBuffer);
        this.ledStrip.start();
    } 

    /**
     * To set ledPattern, method taking in a LEDConfiguration value.
     * 
     * @param ledConfiguration The LEDConfiguration this LEDManager's LED strip will display.
     */
    public void LEDPatternSet(LEDConfiguration ledConfiguration) {
        ledConfiguration.getLEDPattern().applyTo(this.ledBuffer);
        this.ledStrip.setData(this.ledBuffer);
    }
}

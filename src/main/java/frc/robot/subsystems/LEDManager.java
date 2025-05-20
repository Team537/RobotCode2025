package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
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

    private static LEDManager instance;

    // Hardware
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    /**
     * returns the singleton object of the LED Manager
     * @return the singleton object
     */
    public static LEDManager getInstance() {
        if (instance == null) {
            throw new IllegalStateException("LEDManager has not been initialized. Call initialize() first.");
        } else {
            return instance;
        }
    }

    /**
     * initialized the LEDManager object using the provided PWM port number and length.
     * 
     * @param port The port number the LED strip is plugged into on the PWM.
     * @param length The number of LEDs in the strip
     */
    public static void initialize(int port, int length) {
        if (instance != null) {
            throw new IllegalStateException("LEDManager has already been initialized.");
        }
        instance = new LEDManager(port, length);
    }

    /**
     * Creates a new LEDManager object using the provided PWM port number and length.
     * 
     * @param port The port number the LED strip is plugged into on the PWM.
     * @param length The number of LEDs in the strip
     */
    private LEDManager(int port, int length) {

        // Create new AddressableLED and AddressableLEDBuffer objects.
        this.ledStrip = new AddressableLED(port);
        this.ledBuffer = new AddressableLEDBuffer(length);
        
        // Setup the LED strip hardware
        this.ledStrip.setLength(this.ledBuffer.getLength());
        this.ledStrip.setData(ledBuffer);
        this.ledStrip.start();
    }

    /**
     * creates a view to control a segment of the led strip
     * @param ledSegment the segment to create the view out of
     * @return the view to control the segment
     */
    public static AddressableLEDBufferView getViewForSegment(LEDSegment ledSegment) {
        return instance.ledBuffer.createView(ledSegment.getStartingIndex(),ledSegment.getEndingIndex());
    }

    /**
     * updates the LED strip.
     */
    public static void update() {
        instance.ledStrip.setData(instance.ledBuffer);
    }

}

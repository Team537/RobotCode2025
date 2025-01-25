package frc.utils.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDManager {
    private AddressableLED ledStrip;
    private AddressableLEDBuffer ledBuffer;

    public LEDManager(int portNum) {
        this.ledStrip = new AddressableLED(portNum);
        ledStrip.setLength(ledBuffer.getLength());
        //set data

        ledStrip.setData(ledBuffer);
        ledStrip.start();
    } 

    //to set ledPattern, method taking in ledconfiguration value
    public void LEDPatternSet(LEDConfiguration ledConfiguration) {
        ledConfiguration.getLEDPattern().applyTo(this.ledBuffer);
        this.ledStrip.setData(this.ledBuffer);
    }
}

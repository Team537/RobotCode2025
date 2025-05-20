package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import frc.robot.util.LEDs.LEDConfiguration;

public class LEDSegment {
    
    private int startingIndex;
    private int endingIndex;

    private AddressableLEDBufferView view;

    public LEDSegment(int startingIndex, int endingIndex) {
        this.startingIndex = startingIndex;
        this.endingIndex = endingIndex;
        view = LEDManager.getViewForSegment(this);   
    }

    public void setPattern(LEDConfiguration configuration) {
        configuration.getLEDPattern().applyTo(view);
    }

    public int getStartingIndex() {
        return startingIndex;
    }

    public int getEndingIndex() {
        return endingIndex;
    }

}

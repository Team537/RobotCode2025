package frc.robot.util.field;

/**
 * Ane num used to represent the 
 * @author Parker Huibregtse
 * @version 3.0.0
 */
public enum AlgaeRemovalPosition {
    AB(true),
    CD(false),
    EF(true),
    GH(false),
    IJ(true),
    KL(false);

    private boolean isTopRow;

    /**
     * Creates a new AlgaeRemovalPosition with a boolean keeping track of whether or not it is located in the top row.
     * 
     * @param isTopRow Whether or not this algae is located in the top row.
     */
    private AlgaeRemovalPosition(boolean isTopRow) {
        this.isTopRow = isTopRow;
    }

    /**
     * Returns whether or not the algae is on the top row of the reef.
     * 
     * @return Whether or not the algae is on the top row of the reef.
     */
    public boolean isTopRow() {
        return isTopRow;
    }

    /**
     * (Deprecated) Returns whether or not the algae is on the bottom row of the reef.
     * 
     * <strong>Deprecated:</strong> This method is deprecated in favor of {@link #isTopRow()}
     * which provides further clarity and has been decided to be used in place of this method.
     * 
     * @return Whether or not the algae is on the bottom row of the reef.
     */
    @Deprecated
    public boolean isBottomRow() {
        return !isTopRow;
    }
}

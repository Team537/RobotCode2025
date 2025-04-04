package frc.robot.util.field;

/**
 * <h2> ReefScoringLocation </h2>
 * An enum value used to represent the different scoring locations along the reef. These values correspond and follow
 * FIRST`s own diagrams and standards.
 * <hr>
 * @see <a href=https://firstfrc.blob.core.windows.net/frc2025/Manual/Sections/2025GameManual-05ARENA.pdf> FRC Reef Layout </a> (PDF Page 6)
 * @author Cameron Myhre
 * @version 3.0.0
 */
public enum ReefScoringLocation {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L;

    /**
     * Returns the next scoring location based on the previous location and a direction.
     *
     * @param previousScoringLocation the current scoring location.
     * @param clockwise               if true, the next location is determined in the clockwise direction;
     *                                if false, counter-clockwise.
     * @return the next ReefScoringLocation.
     */
    public static ReefScoringLocation getNextScoringLocation(ReefScoringLocation previousScoringLocation, boolean clockwise) {
        ReefScoringLocation[] allLocations = ReefScoringLocation.values();
        int change = clockwise ? -1 : 1; // positions are alphabetically counter-clockwise
        
        // total length (12) + current position (0-11) +/- 1, modded by 12 eg
        // A->B - (12 + 0 + 1) % 12 = 1
        // A->L - (12 + 0 - 1) % 12 = 11
        // L->A - (12 + 11 + 1) % 12 = 0
        // L->K - (12 + 11 - 1) % 12 = 10
        int nextPosition = (allLocations.length + previousScoringLocation.ordinal() + change) % allLocations.length;
        return allLocations[nextPosition];
    }

    /**
     * Returns the next scoring location from the current one.
     *
     * @param clockwise if true, the next location is determined in the clockwise direction;
     *                  if false, counter-clockwise.
     * @return the next ReefScoringLocation.
     */
    public ReefScoringLocation getNextScoringLocation(boolean clockwise) {
        return getNextScoringLocation(this, clockwise);
    }
}

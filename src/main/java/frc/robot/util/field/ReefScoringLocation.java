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

    public static ReefScoringLocation getNextScoringLocation(ReefScoringLocation previouScoringLocation, boolean clockwise) {
        ReefScoringLocation[] allLocations = ReefScoringLocation.values();
        int change = clockwise ? -1 : 1; // positions are alphabetically counter-clockwise

        // total length (12) + current position (0-11) +/- 1, modded by 12 eg
        // A->B - (12 + 0 + 1) % 12 = 1
        // A->L - (12 + 0 - 1) % 12 = 11
        // L->A - (12 + 11 + 1) % 12 = 0
        // L->K - (12 + 11 - 1) % 12 = 10
        int nextPosition = (allLocations.length + previouScoringLocation.ordinal() + change) % allLocations.length;
        return allLocations[nextPosition];
    }
}

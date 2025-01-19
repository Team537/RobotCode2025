package frc.utils.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * <p>
 * <strong>ScoringLocation</strong> is a data container representing a single
 * scoring position on the field. It has:
 * </p>
 * <ul>
 * <li><code>branch</code>: e.g., "B1", "B2"</li>
 * <li><code>level</code>: e.g., "L2", "L3", "L4"</li>
 * <li><code>transform</code>: The 3D transform (position + orientation).</li>
 * </ul>
 * 
 * @author Cameron Myhre
 * @since v1.2.0
 */
public class ScoringLocation {
    
    /** The branch name (e.g., "B1"). */
    public final String branch;
    /** The level (e.g., "L2"). */
    public final String level;
    /** The 3D transform specifying position (x, y, z) and rotation. */
    public final Transform3d transform;

    /**
     * Constructs a new ScoringLocation with the given identifiers and transform.
     *
     * @param branch    The branch identifier (e.g. "B1").
     * @param level     The level identifier (e.g. "L2").
     * @param transform A 3D transform with (x, y, z) position (and optional
     *                  rotation).
     */
    public ScoringLocation(String branch, String level, Transform3d transform) {
        this.branch = branch;
        this.level = level;
        this.transform = transform;
    }

    /**
     * Returns a string representation for debugging, e.g. "B1-L2 @ (1.00, 0.00,
     * 2.00)".
     *
     * @return A simple string format describing this location.
     */
    @Override
    public String toString() {
        return String.format("%s-%s @ (%.2f, %.2f, %.2f)",
                branch,
                level,
                transform.getX(), //
                transform.getY(),
                transform.getZ());
    }
}
package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.field.ReefScoringLocation;
import frc.robot.util.upper_assembly.ScoringHeight;

/**
 * <p>
 * ScoringLocation is a data container representing a single scoring position on the field.
 * It contains:
 * </p>
 * <ul>
 *   <li><code>branch</code>: e.g., "B1", "B2"</li>
 *   <li><code>level</code>: e.g., "L2", "L3", "L4"</li>
 *   <li><code>transform</code>: The 3D transform specifying position (x, y, z) and rotation.</li>
 *   <li><code>timestamp</code>: The time (in nanoseconds) when this detection data was received.</li>
 * </ul>
 * <hr>
 * @author Cameron Myhre
 * @since v1.2.0
 */
public class ScoringLocation {
    
    public final String branch;
    public final String level;
    public final Transform3d transform;
    public final long timestamp;

    /**
     * Constructs a new ScoringLocation with the given identifiers, transform, and timestamp.
     *
     * @param branch    The branch identifier (e.g. "B1").
     * @param level     The level identifier (e.g. "L2").
     * @param transform A 3D transform with (x, y, z) position and rotation.
     * @param timestamp The time (in nanoseconds) when the detection was received.
     */
    public ScoringLocation(String branch, String level, Transform3d transform, long timestamp) {
        this.branch = branch;
        this.level = level;
        this.transform = transform;
        this.timestamp = timestamp;
    }

    /**
     * Returns this ScoringLocation`s branch as a <code> ReefScoringLocation </code> enum.
     * 
     * @return This ScoringLocation`s branch as a <code> ReefScoringLocation </code> enum.
     */
    public ReefScoringLocation getBranchAsEnum() {
        return ReefScoringLocation.valueOf(this.branch);
    }

    /**
     * Returns this ScoringLocation`s level as a <code> ScoringHeight </code> enum.
     * 
     * @return This ScoringLocation`s level as a <code> ScoringHeight </code> enum.
     */
    public ScoringHeight getLevelAsEnum() {
        return ScoringHeight.valueOf(this.level);
    }

    /**
     * Returns a string representation for debugging, e.g. "B1-L2 @ (1.00, 0.00,
     * 2.00)".
     *
     * @return A formatted string describing this scoring location.
     */
    @Override
    public String toString() {
        return String.format("%s-%s @ (%.2f, %.2f, %.2f) [ts=%d]",
                branch,
                level,
                transform.getX(), 
                transform.getY(),
                transform.getZ(),
                timestamp);
    }
}
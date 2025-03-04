package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OceanViewConstants;
import frc.robot.network.TCPSender;
import frc.robot.network.UDPReceiver;
import frc.robot.util.upper_assembly.ScoringHeight;
import frc.robot.util.vision.ScoringLocation;

import java.io.IOException;
import java.util.*;
import java.util.function.Supplier;

/**
 * <h1>OceanViewManager</h1>
 * <p>
 * The <strong>OceanViewManager</strong> class manages detection data received from a
 * Raspberry Pi (or another device) over UDP, parses it, stores it, and provides
 * methods to retrieve both <em>current</em> detection data and a short-term
 * <strong>memory</strong> of old detection data. 
 * </p>
 *
 * <h2>JSON Format</h2>
 * <p>
 * We assume the Raspberry Pi sends a JSON structure of the form:
 * </p>
 * <pre>
 * {
 *   "available": [
 *     {"branch": "B1", "level": "L2", "position": {"x":1.0, "y":0.0, "z":2.0}},
 *     ...
 *   ],
 *   "algae_blocked": [
 *     {"branch": "B3", "level": "L2", "position": {"x":1.5, "y":0.0, "z":2.0}},
 *     ...
 *   ],
 *   "algae_positions": [
 *     {"x":1.0, "y":0.0, "z":2.0},
 *     ...
 *   ],
 *   "packet_number": 42
 * }
 * </pre>
 *
 * <h2>Features</h2>
 * <ul>
 *   <li><strong>Exponential Smoothing</strong> on (x, y, z) to reduce noise.</li>
 *   <li><strong>Rolling Memory</strong> of up to N old transforms per scoring location,
 *       to avoid discarding older data entirely.</li>
 *   <li>Simple <strong>Filtering by Level</strong> (L2, L3, L4) with dedicated methods.</li>
 *   <li>Convenient methods to compute a <strong>rolling average</strong> of the memory
 *       for advanced noise reduction.</li>
 * </ul>
 * <hr>
 * @author Cameron Myhre
 * @since v2.0.0
 */
public class OceanViewManager extends SubsystemBase {

    // ------------------------------------------------------------------------
    // Current Frame Data
    // ------------------------------------------------------------------------

    /**
     * <p>A list of <strong>unblocked</strong> scoring locations as parsed from the JSON "available" array.</p>
     */
    private final List<ScoringLocation> availableLocations = new ArrayList<>();

    /**
     * <p>A list of scoring locations <strong>blocked by algae</strong> from the JSON "algae_blocked" array.</p>
     */
    private final List<ScoringLocation> algaeBlockedLocations = new ArrayList<>();

    /**
     * <p>A list of generic algae detections (x,y,z only) from the JSON "algae_positions" array.</p>
     */
    private final List<Transform3d> algaePositions = new ArrayList<>();

    // ------------------------------------------------------------------------
    // Packet Tracking
    // ------------------------------------------------------------------------

    /**
     * <p>Tracks the last <em>packet_number</em> we processed. This helps ensure 
     * we only parse new data when the Pi sends an updated packet.</p>
     */
    private int lastProcessedPacket = -1;

    // ------------------------------------------------------------------------
    // Exponential Smoothing + Rolling Memory
    // ------------------------------------------------------------------------

    /**
     * <p>
     * A map of <code>(branch-level)</code> -> previously smoothed transform,
     * used for exponential smoothing each time we see new data for that ID.
     * </p>
     * <p>
     * Key example: "B1-L2", Value: the last <code>Transform3d</code> we used.
     * </p>
     */
    private final Map<String, Transform3d> previousTransforms = new HashMap<>();

    /**
     * <p>
     * <strong>detectionHistory</strong> stores a short memory (queue) of old transforms
     * for each scoring location ID, to avoid losing older detections entirely.
     * </p>
     * <p>
     * Key: "B1-L2", Value: a queue (FIFO) of <code>Transform3d</code> objects representing
     * recent detections.
     * </p>
     */
    private final Map<String, Deque<Transform3d>> detectionHistory = new HashMap<>();

    /**
     * <p>
     * The maximum number of old transforms to keep in memory for each scoring location ID.
     * If a queue is full, the oldest item is removed before adding the newest.
     * </p>
     */
    private final int maxMemorySize = 5;

    /**
     * <p>
     * Exponential smoothing factor. A value between 0 and 1:
     * </p>
     * <ul>
     *   <li><strong>Closer to 1</strong>: minimal smoothing (fast response).</li>
     *   <li><strong>Closer to 0</strong>: heavier smoothing (less noise, but slower to update).</li>
     * </ul>
     */
    private static final double SMOOTHING_ALPHA = 0.3;

    // ------------------------------------------------------------------------
    // Network Fields
    // ------------------------------------------------------------------------

    /**
     * <p>The <strong>UDPReceiver</strong> that collects JSON data from the Raspberry Pi over UDP.</p>
     */
    private final UDPReceiver udpReceiver;

    /**
     * <p>The <strong>TCPSender</strong> for sending data/commands to the Pi, if desired. </p>
     */
    private final TCPSender tcpSender;

    /**
     *<p> The <strong>supplier method</strong> for getting the robots position. Used
     * to send the robot's position to the PI for operational use </p>
     */
    private final Supplier<Pose2d> poseSupplier;

    // ------------------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------------------

    /**
     * Constructs a new OceanViewManager with the specified network devices.
     *
     * @param udpReceiver  The UDPReceiver for fetching JSON detection data.
     * @param tcpSender    A TCPSender if you need to send data to the Pi.
     * @param poseSupplier A supplier method that returns the robot's estimated position.
     */
    public OceanViewManager(UDPReceiver udpReceiver, TCPSender tcpSender, Supplier<Pose2d> poseSupplier) {
        this.udpReceiver  = udpReceiver;
        this.tcpSender    = tcpSender;
        this.poseSupplier = poseSupplier;
    }

    // ------------------------------------------------------------------------
    // Periodic
    // ------------------------------------------------------------------------

    /**
     * <p>
     * The <strong>periodic()</strong> method is called roughly every 20ms by WPILib.
     * We use it to check if a new <code>packet_number</code> has arrived via UDP.
     * If so, we parse the new data.
     * </p>
     */
    @Override
    public void periodic() {

        // Check the latest packet_number
        int currentPacket = this.udpReceiver.getPacketNumber();

        // If we have a new packet_number, parse new data
        if (currentPacket > lastProcessedPacket) {
            lastProcessedPacket = currentPacket;
            fetchDetectionData();
        }

        // Update the robots position on the PI. If this ends up being too much, we can 
        // always decrease the time between sends.
        sendRobotPoseToPi();
    }

    /**
     * Packages and sends the robot's current pose to the Raspberry Pi over TCP.
     */
    private void sendRobotPoseToPi() {
        if (tcpSender == null || !tcpSender.isConnected()) {
            // Attempt to reconnect or log a warning
            try {
                tcpSender.reconnect(OceanViewConstants.PI_IP, OceanViewConstants.TCP_PORT_NUMBER); // Replace with actual IP and port
                System.out.println("[OceanViewManager] Reconnected to Raspberry Pi.");
            } catch (IOException e) {
                System.err.println("[OceanViewManager] Failed to reconnect: " + e.getMessage());
                return;
            }
        }

        // Get the current estimated robot pose and format it such that the data is easily manipulated by GSON.
        Pose2d currentPose = poseSupplier.get();
        RobotPoseData data = new RobotPoseData(
            currentPose.getX(),
            currentPose.getY(),
            currentPose.getRotation().getRadians(),
            Timer.getFPGATimestamp()
        );

        tcpSender.sendConfiguration(data);
    }

    // ------------------------------------------------------------------------
    // Main Parsing & Data Handling
    // ------------------------------------------------------------------------

    /**
     * <p>
     * <strong>fetchDetectionData()</strong> retrieves the latest JSON from the
     * <code>udpReceiver</code>, clears out old lists, parses the new data,
     * applies smoothing and updates rolling memory, and populates
     * <code>availableLocations</code>, <code>algaeBlockedLocations</code>,
     * and <code>algaePositions</code>.
     * </p>
     */
    private void fetchDetectionData() {
        // Retrieve a Map<String, Object> representing the entire JSON
        Map<String, Object> jsonData = udpReceiver.getTargetData();

        // Clear old data from the previous cycle
        availableLocations.clear();
        algaeBlockedLocations.clear();
        algaePositions.clear();

        // If there's no data, we can't parse anything
        if (jsonData == null || jsonData.isEmpty()) {
            System.out.println("[OceanViewManager] No data or empty JSON. Skipping parse.");
            return;
        }

        // Parse each major JSON key
        parseLocationArray(jsonData, "available",     availableLocations);
        parseLocationArray(jsonData, "algae_blocked", algaeBlockedLocations);
        parseAlgaePositions(jsonData, "algae_positions", algaePositions);

        // Optional debug logs
        System.out.println("[OceanViewManager] Data updated:");
        System.out.printf("  Available: %d, Blocked: %d, AlgaePts: %d%n",
            availableLocations.size(),
            algaeBlockedLocations.size(),
            algaePositions.size()
        );
    }

    // ------------------------------------------------------------------------
    // JSON Parsing Helpers
    // ------------------------------------------------------------------------

    /**
     * <p>
     * Parses an array of scoring locations under a specific key, e.g. "available" or "algae_blocked".
     * </p>
     * <p>
     * Each element is expected to look like:
     * </p>
     * <pre>
     * {
     *   "branch": "B1",
     *   "level":  "L2",
     *   "position": {"x":1.0, "y":0.0, "z":2.0}
     * }
     * </pre>
     *
     * @param jsonData   The overall JSON map from the Pi.
     * @param key        The array key, e.g. "available" or "algae_blocked".
     * @param outputList The list to store parsed <code>ScoringLocation</code> objects.
     */
    @SuppressWarnings("unchecked")
    private void parseLocationArray(Map<String, Object> jsonData, String key, List<ScoringLocation> outputList) {
        
        // Extract the raw array
        Object rawObj = jsonData.get(key);
        if (!(rawObj instanceof List)) {
            return; // Key not present or not a list, skip
        }

        List<Object> rawList = (List<Object>) rawObj;

        // Iterate over each element in the list
        for (Object o : rawList) {
            if (!(o instanceof Map)) {
                continue; // Not the expected structure, skip
            }

            // Each element is a map with "branch", "level", and "position"
            Map<String, Object> locMap = (Map<String, Object>) o;
            String branch = safeGetString(locMap, "branch", "UnknownBranch");
            String level  = safeGetString(locMap, "level",  "UnknownLevel");

            // Nested position
            Map<String, Object> posMap = (Map<String, Object>) locMap.get("position");
            if (posMap == null) {
                continue; // If no position map is found, skip
            }

            // Extract x, y, z
            double x = safeGetDouble(posMap, "x", 0.0);
            double y = safeGetDouble(posMap, "y", 0.0);
            double z = safeGetDouble(posMap, "z", 0.0);

            // Apply exponential smoothing
            String id = branch + "-" + level;
            Transform3d smoothed = smoothPosition(id, x, y, z);

            // Add to outputList
            ScoringLocation location = new ScoringLocation(branch, level, smoothed);
            outputList.add(location);

            // Also store in rolling memory
            storeInDetectionHistory(id, smoothed);
        }
    }

    /**
     * <p>
     * Parses an array of algae positions under "algae_positions", each of which 
     * is just (x, y, z) with no branch or level.
     * </p>
     *
     * @param jsonData   The entire JSON map.
     * @param key        Usually "algae_positions".
     * @param outputList The list to store <code>Transform3d</code> objects.
     */
    @SuppressWarnings("unchecked")
    private void parseAlgaePositions(Map<String, Object> jsonData, String key, List<Transform3d> outputList) {
        Object rawObj = jsonData.get(key);
        if (!(rawObj instanceof List)) {
            return;
        }

        List<Object> rawList = (List<Object>) rawObj;
        for (Object o : rawList) {
            if (!(o instanceof Map)) {
                continue;
            }

            Map<String, Object> posMap = (Map<String, Object>) o;
            double x = safeGetDouble(posMap, "x", 0.0);
            double y = safeGetDouble(posMap, "y", 0.0);
            double z = safeGetDouble(posMap, "z", 0.0);

            // We do not apply exponential smoothing for algaePositions
            Transform3d algaeTransform = new Transform3d(x, y, z, new Rotation3d());
            outputList.add(algaeTransform);
        }
    }

    // ------------------------------------------------------------------------
    // Smoothing & Memory
    // ------------------------------------------------------------------------

    /**
     * <p>
     * Performs exponential smoothing on the new (x,y,z) for a given ID. If no 
     * previous transform exists, we simply store the new data as-is.
     * </p>
     *
     * @param id   Unique identifier (e.g., "B1-L2").
     * @param newX Newly measured x coordinate.
     * @param newY Newly measured y coordinate.
     * @param newZ Newly measured z coordinate.
     * @return A <code>Transform3d</code> that is the smoothed position.
     */
    private Transform3d smoothPosition(String id, double newX, double newY, double newZ) {

        // Check for existing old transform
        Transform3d oldTransform = previousTransforms.get(id);
        if (oldTransform == null) {

            // No old data => store new as-is
            Transform3d fresh = new Transform3d(newX, newY, newZ, new Rotation3d());
            previousTransforms.put(id, fresh);
            return fresh;
        }

        // Exponential smoothing formula
        double oldX = oldTransform.getX();
        double oldY = oldTransform.getY();
        double oldZ = oldTransform.getZ();

        double smoothedX = SMOOTHING_ALPHA * newX + (1.0 - SMOOTHING_ALPHA) * oldX;
        double smoothedY = SMOOTHING_ALPHA * newY + (1.0 - SMOOTHING_ALPHA) * oldY;
        double smoothedZ = SMOOTHING_ALPHA * newZ + (1.0 - SMOOTHING_ALPHA) * oldZ;

        Transform3d result = new Transform3d(smoothedX, smoothedY, smoothedZ, new Rotation3d());

        // Update the stored transform for next iteration
        previousTransforms.put(id, result);

        return result;
    }

    /**
     * <p>
     * Stores the given <code>Transform3d</code> in a short-term memory queue
     * for the specified scoring location ID. This queue is capped at <code>maxMemorySize</code>.
     * </p>
     *
     * @param id        A unique ID (e.g., "B1-L2").
     * @param transform The final transform (already smoothed) for this update cycle.
     */
    private void storeInDetectionHistory(String id, Transform3d transform) {

        // Retrieve or create the queue for this ID
        Deque<Transform3d> queue = detectionHistory.get(id);
        if (queue == null) {
            queue = new ArrayDeque<>();
            detectionHistory.put(id, queue);
        }

        // If we're at capacity, remove the oldest
        if (queue.size() >= maxMemorySize) {
            queue.removeFirst();
        }

        // Add the newest transform
        queue.addLast(transform);
    }

    /**
     * <p>
     * Optionally computes a rolling average of the last few transforms (up to 
     * <code>maxMemorySize</code>) for a given ID. This is an alternative (or supplement)
     * to exponential smoothing if you want to incorporate multiple recent detections.
     * </p>
     *
     * @param id The ID of the scoring location (e.g., "B1-L2").
     * @return A <code>Transform3d</code> representing the average of all stored transforms, 
     *         or <code>null</code> if no data is available.
     */
    public Transform3d getRollingAverageTransform(String id) {

        // Get the queue of old transforms for this ID
        Deque<Transform3d> queue = detectionHistory.get(id);
        if (queue == null || queue.isEmpty()) {
            return null;
        }

        // Sum up all x, y, z
        double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
        for (Transform3d t : queue) {
            sumX += t.getX();
            sumY += t.getY();
            sumZ += t.getZ();
        }

        // Compute average
        int count = queue.size();
        double avgX = sumX / count;
        double avgY = sumY / count;
        double avgZ = sumZ / count;

        // Return as a new Transform3d (no rotation averaging here, but could be extended)
        return new Transform3d(avgX, avgY, avgZ, new Rotation3d());
    }

    /**
     * <p>
     * Retrieves the raw queue (in a read-only form) of old transforms stored
     * for a given ID, so you can apply additional logic (e.g. outlier rejection).
     * </p>
     *
     * @param id The scoring location ID (e.g., "B1-L3").
     * @return An unmodifiable list of transforms, or an empty list if none found.
     */
    public List<Transform3d> getDetectionHistoryFor(String id) {
        Deque<Transform3d> queue = detectionHistory.get(id);
        if (queue == null) {
            return Collections.emptyList();
        }

        // Create a snapshot list and wrap in unmodifiable
        return Collections.unmodifiableList(new ArrayList<>(queue));
    }

    // ------------------------------------------------------------------------
    //   Safe Accessors
    // ------------------------------------------------------------------------ 

    /**
     * Retrieves a string from a map. If the key is missing or not a string, returns defaultVal.
     *
     * @param map        The map to check.
     * @param key        The key in the map.
     * @param defaultVal A fallback if the key is missing.
     * @return The string value or defaultVal.
     */
    private String safeGetString(Map<String, Object> map, String key, String defaultVal) {
        Object val = map.get(key);
        if (val instanceof String) {
            return (String) val;
        }
        return defaultVal;
    }

    /**
     * Retrieves a double from a map. If the key is missing or not numeric, returns defaultVal.
     *
     * @param map        The map to check.
     * @param key        The key in the map.
     * @param defaultVal A fallback if the key is missing.
     * @return The double value or defaultVal.
     */
    private double safeGetDouble(Map<String, Object> map, String key, double defaultVal) {
        Object val = map.get(key);
        if (val instanceof Number) {
            return ((Number) val).doubleValue();
        }
        return defaultVal;
    }

    // ------------------------------------------------------------------------
    // Public Accessors for Current Frame Data
    // ------------------------------------------------------------------------

    /**
     * @return An unmodifiable list of <strong>unblocked</strong> scoring locations 
     *         ("available") from the most recent data frame.
     */
    public List<ScoringLocation> getAllAvailableLocations() {
        return Collections.unmodifiableList(availableLocations);
    }

    /**
     * @return An unmodifiable list of <strong>algae-blocked</strong> scoring locations 
     *         from the most recent data frame.
     */
    public List<ScoringLocation> getAllAlgaeBlockedLocations() {
        return Collections.unmodifiableList(algaeBlockedLocations);
    }

    /**
     * @return An unmodifiable list of <code>Transform3d</code> objects representing 
     *         algae positions from the most recent data frame.
     */
    public List<Transform3d> getAllAlgaePositions() {
        return Collections.unmodifiableList(algaePositions);
    }

    /**
     * <p>
     * (Deprecated) Retrieves all <strong>available scoring locations</strong> for a given level,
     * such as "L2", "L3", or "L4".
     * </p>
     * <strong>Deprecated:</strong> This method is deprecated in favor of {@link #getAvailableByLevel(ScoringHeight)}
     * which provides improved type safety and clarity by using the <code>ScoringHeight</code> type rather than a raw String.
     * 
     * @param level The desired level (e.g., "L2"). {@link #getAvailableByLevel(ScoringHeight)}
     * @return A list of matching <code>ScoringLocation</code> objects.
     */
    @Deprecated
    public List<ScoringLocation> getAvailableByLevel(String level) {
        List<ScoringLocation> result = new ArrayList<>();
        for (ScoringLocation loc : availableLocations) {
            if (loc.level.equalsIgnoreCase(level)) {
                result.add(loc);
            }
        }
        return result;
    }

    /**
     * <p>
     * Retrieves all <strong>available scoring locations</strong> for a given level,
     * such as "L2", "L3", or "L4".
     * </p>
     *
     * @param level The scoring level filter as a <code>ScoringHeight</code> (e.g. <code>ScoringHeight.L2</code>).
     * @return A list of matching <code>ScoringLocation</code> objects.
     */
    public List<ScoringLocation> getAvailableByLevel(ScoringHeight level) {

        // Get the string form of the scoring height.
        String levelString = level.toString();

        // Get the blocked locations by scoring height.
        List<ScoringLocation> result = new ArrayList<>();
        for (ScoringLocation loc : availableLocations) {
            if (loc.level.equalsIgnoreCase(levelString)) {
                result.add(loc);
            }
        }
        return result;
    }

    /**
     * <p>
     * (Deprecated) Retrieves all <strong>algae-blocked scoring locations</strong> for a given level,
     * e.g., "L2".
     * </p>
     * <strong>Deprecated:</strong> This method is deprecated in favor of {@link #getBlockedByLevel(ScoringHeight)}
     * which provides improved type safety and clarity by using the <code>ScoringHeight</code> type rather than a raw String.
     * 
     * @param level The desired level (e.g., "L3"). Use {@link #getBlockedByLevel(ScoringHeight)} instead.
     * @return A list of blocked <code>ScoringLocation</code> objects at that level.
     */
    @Deprecated
    public List<ScoringLocation> getBlockedByLevel(String level) {
        List<ScoringLocation> result = new ArrayList<>();
        for (ScoringLocation loc : algaeBlockedLocations) {
            if (loc.level.equalsIgnoreCase(level)) {
                result.add(loc);
            }
        }
        return result;
    }

    /**
     * <p>
     * Retrieves all <strong>algae-blocked scoring locations</strong> for a given level,
     * e.g., "L2".
     * </p>
     *
     * @param level The scoring level filter as a <code>ScoringHeight</code> (e.g. <code>ScoringHeight.L2</code>).
     * @return A list of blocked <code>ScoringLocation</code> objects at that level.
     */
    public List<ScoringLocation> getBlockedByLevel(ScoringHeight level) {

        // Get the string form of the scoring height.
        String levelString = level.toString();

        // Get the blocked locations by scoring height.
        List<ScoringLocation> result = new ArrayList<>();
        for (ScoringLocation loc : algaeBlockedLocations) {
            if (loc.level.equalsIgnoreCase(levelString)) {
                result.add(loc);
            }
        }
        return result;
    }

    /**
     * <p>
     * (Deprecated) Returns both <strong>available</strong> and <strong>blocked</strong> scoring
     * locations for the specified level in a single call.
     * </p>
     * <p>
     * <strong>Deprecated:</strong> This method is deprecated in favor of {@link #getAllLocationsByLevel(ScoringHeight)}
     * which provides improved type safety and clarity by using the <code>ScoringHeight</code> type rather than a raw String.
     * </p>
     *
     * @param level The level filter as a String (e.g. "L2"). Use {@link #getAllLocationsByLevel(ScoringHeight)} instead.
     * @return A map with two keys: "available" and "blocked", each mapping to a list of <code>ScoringLocation</code>.
     */
    @Deprecated
    public Map<String, List<ScoringLocation>> getAllLocationsByLevel(String level) {
        Map<String, List<ScoringLocation>> result = new HashMap<>();
        result.put("available", getAvailableByLevel(level));
        result.put("blocked",   getBlockedByLevel(level));
        return result;
    }

    /**
     * <p>
     * Returns both <strong>available</strong> and <strong>blocked</strong> scoring locations for the specified level
     * in a single call.
     * </p>
     * <p>
     * This is the recommended method to use as it accepts a <code>ScoringHeight</code> parameter, which enhances type safety
     * and reduces errors compared to the deprecated String‑based method.
     * </p>
     *
     * @param level The scoring level filter as a <code>ScoringHeight</code> (e.g. <code>ScoringHeight.L2</code>).
     * @return A map with two keys: "available" and "blocked", each mapping to a list of <code>ScoringLocation</code>.
     */
    public Map<String, List<ScoringLocation>> getAllLocationsByLevel(ScoringHeight level) {

        // Get the string form of the scoring height.
        String levelString = level.toString();
        
        // Get all scoring locations by the scoring level.
        Map<String, List<ScoringLocation>> result = new HashMap<>();
        result.put("available", getAvailableByLevel(levelString));
        result.put("blocked",   getBlockedByLevel(levelString));
        return result;
    }
}
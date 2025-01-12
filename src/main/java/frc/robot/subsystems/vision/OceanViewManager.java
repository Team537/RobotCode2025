package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.network.TCPSender;
import frc.robot.network.UDPReceiver;

/**
 * OceanViewManager is responsible for managing detection data received from
 * the network, parsing it, and storing it for further processing.
 */
public class OceanViewManager extends SubsystemBase {

    // Storage for target positions
    private final List<Transform3d> algaePositions = new ArrayList<>();
    private final List<Transform3d> l2Positions = new ArrayList<>();
    private final List<Transform3d> l3Positions = new ArrayList<>();
    private final List<Transform3d> l4Positions = new ArrayList<>();

    // Network components
    private final UDPReceiver udpReceiver;
    private final TCPSender tcpSender;

    // Tracking the latest data packet number
    private int lastProcessedPacket = -1;

    /**
     * Constructs a new OceanViewManager.
     * 
     * @param udpReceiver The UDPReceiver to fetch detection data.
     * @param tcpSender   The TCPSender to send commands or data (if needed).
     */
    public OceanViewManager(UDPReceiver udpReceiver, TCPSender tcpSender) {
        this.udpReceiver = udpReceiver;
        this.tcpSender = tcpSender;
    }

    /**
     * Fetches detection data from the network, parses it, and updates the
     * position lists.
     */
    public void fetchDetectionData() {
        // Clear previous detection data to ensure updated results
        clearPositionData();

        // Retrieve detection data from UDPReceiver
        Map<String, Object> targets = udpReceiver.getTargetData();

        // Parse and update position data for each target category
        updatePositionData("L2", targets, l2Positions);
        updatePositionData("L3", targets, l3Positions);
        updatePositionData("L4", targets, l4Positions);
        updatePositionData("algae", targets, algaePositions);
    }

    /**
     * Clears all stored position data for targets.
     */
    private void clearPositionData() {
        l2Positions.clear();
        l3Positions.clear();
        l4Positions.clear();
        algaePositions.clear();
    }

    /**
     * Updates the positions list for a specific target category.
     * 
     * @param key          The key in the target data corresponding to the category.
     * @param targets      The raw target data from the UDPReceiver.
     * @param positionsList The list to store parsed positions.
     */
    private void updatePositionData(String key, Map<String, Object> targets, List<Transform3d> positionsList) {
        if (targets != null && targets.containsKey(key)) {
            List<Map<String, Double>> data = parseJsonArray(targets.get(key));
            positionsList.addAll(parseScoringData(data));
        } else {
            System.out.printf("%s data not found in targets.%n", key);
        }
    }

    /**
     * Parses a raw JSON array into a list of maps with numerical values.
     * 
     * @param data The raw data object to parse.
     * @return A list of maps representing parsed data, or an empty list if parsing
     *         fails.
     */
    @SuppressWarnings("unchecked")
    private List<Map<String, Double>> parseJsonArray(Object data) {
        try {
            return (List<Map<String, Double>>) data;
        } catch (ClassCastException e) {
            System.err.printf("Error casting JSON data for key: %s%n", e.getMessage());
            return List.of(); // Return an empty list on failure
        }
    }

    /**
     * Converts raw position data into a list of Transform3d objects.
     * 
     * @param data The raw position data to convert.
     * @return A list of Transform3d objects representing positions.
     */
    private List<Transform3d> parseScoringData(List<Map<String, Double>> data) {
        List<Transform3d> positions = new ArrayList<>();

        for (Map<String, Double> positionValues : data) {
            // Safely retrieve position values and handle missing data
            double x = positionValues.getOrDefault("x", 0.0);
            double y = positionValues.getOrDefault("y", 0.0);
            double z = positionValues.getOrDefault("z", 0.0);

            // Create a Transform3d object and add it to the list
            Transform3d position = new Transform3d(x, y, z, new Rotation3d());
            positions.add(position);
        }

        return positions;
    }

    /**
     * Periodically checks for new data packets and updates detection data if a new
     * packet is available.
     */
    @Override
    public void periodic() {
        int currentPacket = udpReceiver.getPacketNumber();
        if (currentPacket > lastProcessedPacket) {
            fetchDetectionData();
            lastProcessedPacket = currentPacket;
        }
    }
}

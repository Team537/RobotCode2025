package frc.robot.network;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.charset.StandardCharsets;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import java.lang.reflect.Type;
import java.util.Map;

/**
 * UDPReceiver is responsible for receiving JSON data from a Raspberry Pi over UDP.
 * This class runs a separate thread to continuously listen for incoming data without blocking the main robot control loop.
 * <hr>
 * @author Cameron Myhre
 * @since v2.0.0
 */
public class UDPReceiver {

    private final Gson gson = new Gson(); // Reuse Gson instance for efficiency
    private final Type targetType = new TypeToken<Map<String, Object>>() {}.getType();
    private volatile Map<String, Object> targets;

    // Storage
    private int portNumber;
    private int previousPacketNumber = 0;

    /**
     * Create a new UDPReceiver object to receive data from  the given port.
     * 
     * @param portNumber The port number data will be received from.
     */
    public UDPReceiver(int portNumber) {
        this.portNumber = portNumber;
    }

    /**
     * Parses and updates the list of targets from a received JSON string.
     *
     * @param jsonString The JSON string containing target data.
     */
    private synchronized void updateTargets(String jsonString) {
        this.targets = gson.fromJson(jsonString, targetType);
    }

    /**
     * Starts the UDP receiver to listen for incoming data on the specified port.
     * This method spawns a new thread to handle incoming packets asynchronously.
     */
    public void start() {
        new Thread(() -> {
            try (DatagramSocket socket = new DatagramSocket(this.portNumber)) {
                byte[] buffer = new byte[4096];
                System.out.println("Waiting for UDP data...");

                while (true) {
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    socket.receive(packet);

                    String jsonString = new String(packet.getData(), 0, packet.getLength(), StandardCharsets.UTF_8);
                    System.out.println("Received JSON: " + jsonString);

                    // Update targets in a thread-safe manner
                    updateTargets(jsonString);
                    
                    // Get the number associated with this packet and check if we lost any data.
                    handlePacketNumber(getPacketNumber());

                    // Add a small sleep to reduce CPU usage
                    Thread.sleep(25);
                }
            } catch (Exception e) {
                System.err.println("Error in UDPReceiver: " + e.getMessage());
                e.printStackTrace();
            }
        }, "UDPReceiverThread").start(); // Name the thread for easier debugging
    }

    /**
     * Handles the packet number to detect and log packet loss.
     *
     * @param currentPacketNumber The packet number of the current packet.
     */
    private synchronized void handlePacketNumber(int currentPacketNumber) {
        if (currentPacketNumber == -1) {
            System.out.println("Invalid packet number received, skipping packet loss check.");
            return;
        }

        if (currentPacketNumber != previousPacketNumber + 1) {
            // Detect packet loss
            System.out.printf("Packet loss detected: Packets %d to %d were lost.%n",
                              previousPacketNumber + 1, currentPacketNumber - 1);
        }

        // Update the previous packet number
        this.previousPacketNumber = currentPacketNumber;
    }

    /**
     * Extracts the packet number from the latest target data.
     *
     * @return The packet number, or -1 if not available or an error occurs.
     */
    public synchronized int getPacketNumber() {

        // If no data has been sent yet, return -1.
        if (this.targets == null || this.targets.isEmpty()) {
            System.out.println("No targets available to extract packet_number.");
            return -1;
        }

        // Assuming the first map in targets contains the packet_number field
        Object packetNumber = targets.get("packet_number"); // Get the first element of the list
        if (packetNumber != null) {

            // Safely parse the packet_number field
            try {
                return ((Number) packetNumber).intValue();
            } catch (ClassCastException e) {
                System.err.println("Error parsing packet_number: " + e.getMessage());
            }
        }

        System.out.println("packet_number not found in targets.");
        return -1; // Default return value if packet_number is not present
    }

    /**
     * Returns the latest list of detected targets.
     * This method is synchronized to ensure thread-safe access to the shared
     * resource.
     *
     * @return A list of targets, or null if no data has been received yet.
     */
    public synchronized Map<String, Object> getTargetData() {
        return targets;
    }
}
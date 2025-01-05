package frc.robot.network;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import frc.robot.Constants.RaspberryPIConstants;

import java.lang.reflect.Type;
import java.util.List;
import java.util.Map;

/**
 * UDPReceiver is responsible for receiving JSON data from a Raspberry Pi over UDP.
 * This class runs a separate thread to continuously listen for incoming data
 * without blocking the main robot control loop.
 */
public class UDPReceiver {

    private volatile List<Map<String, Object>> targets; // Use volatile for visibility across threads
    private final Gson gson = new Gson(); // Reuse Gson instance for efficiency
    private final Type targetType = new TypeToken<List<Map<String, Object>>>() {}.getType(); // Reuse type for deserialization

    /**
     * Parses and updates the list of targets from a received JSON string.
     *
     * @param jsonString The JSON string containing target data.
     */
    private synchronized void updateTargets(String jsonString) {
        targets = gson.fromJson(jsonString, targetType);
    }

    /**
     * Starts the UDP receiver to listen for incoming data on the specified port.
     * This method spawns a new thread to handle incoming packets asynchronously.
     */
    public void start() {
        new Thread(() -> {
            try (DatagramSocket socket = new DatagramSocket(RaspberryPIConstants.PORT_NUMBER)) {
                byte[] buffer = new byte[1024];
                System.out.println("Waiting for UDP data...");

                while (true) {
                    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                    socket.receive(packet);

                    String jsonString = new String(packet.getData(), 0, packet.getLength());
                    System.out.println("Received JSON: " + jsonString);

                    // Update targets in a thread-safe manner
                    updateTargets(jsonString);

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
     * Returns the latest list of detected targets.
     * This method is synchronized to ensure thread-safe access to the shared resource.
     *
     * @return A list of targets, or null if no data has been received yet.
     */
    public synchronized List<Map<String, Object>> getTargets() {
        return targets;
    }
}

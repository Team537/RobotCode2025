package frc.robot.network;

import java.net.*;
import com.google.gson.Gson;
import java.util.ArrayList;
import java.util.List;

/**
 * TimeSyncClient performs time synchronization with the Raspberry Pi using UDP.
 * It sends multiple time sync requests using nanosecond timestamps (via System.nanoTime())
 * and calculates the average clock offset and round-trip delay.
 *
 * The offset is computed using:
 *   offset = ((T2 - T1) + (T3 - T4)) / 2
 * where T1 is the send time and T4 is the receive time on the RoboRIO.
 * <hr>
 * @author Cameron Myhre
 * @version 3.0.0
 */
public class TimeSyncClient {
    private final String piIp;
    private final int piPort;
    private final int numSamples;
    private final Gson gson;
    
    /**
     * Container for time sync response data from the Raspberry Pi.
     * All times are in nanoseconds.
     */
    public static class TimeSyncResponse {
        public long t2; // Pi receive time
        public long t3; // Pi send time
    }
    
    /**
     * Constructs a TimeSyncClient.
     * 
     * @param piIp      The IP address of the Raspberry Pi.
     * @param piPort    The port number on which the Pi’s time sync server listens.
     * @param numSamples The number of UDP requests to send for averaging.
     */
    public TimeSyncClient(String piIp, int piPort, int numSamples) {
        this.piIp = piIp;
        this.piPort = piPort;
        this.numSamples = numSamples;
        this.gson = new Gson();
    }
    
    /**
     * Sends several UDP time sync requests and computes the average clock offset and round-trip delay.
     * Timestamps are obtained in nanoseconds.
     *
     * @return a double array: [average offset (ns), average round-trip delay (ns)]
     */
    public double[] synchronizeTime() {
        List<Long> offsets = new ArrayList<>();
        List<Long> delays = new ArrayList<>();
        
        try (DatagramSocket socket = new DatagramSocket()) {
            socket.setSoTimeout(1000); // 1 second timeout
            byte[] receiveBuffer = new byte[2048];
            
            for (int i = 0; i < numSamples; i++) {
                // Record send time T1 in nanoseconds
                long t1 = System.nanoTime();
                
                // Send a dummy time sync request
                String request = "TIME_SYNC";
                byte[] sendData = request.getBytes("UTF-8");
                DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, InetAddress.getByName(piIp), piPort);
                socket.send(sendPacket);
                
                // Wait for a response and record receive time T4 in nanoseconds
                DatagramPacket receivePacket = new DatagramPacket(receiveBuffer, receiveBuffer.length);
                socket.receive(receivePacket);
                long t4 = System.nanoTime();
                
                // Parse JSON response into TimeSyncResponse object
                String jsonResponse = new String(receivePacket.getData(), 0, receivePacket.getLength(), "UTF-8");
                TimeSyncResponse response = gson.fromJson(jsonResponse, TimeSyncResponse.class);
                
                // Calculate round-trip delay and clock offset (ignoring processing time)
                long delay = (t4 - t1) - (response.t3 - response.t2);
                long offset = ((response.t2 - t1) + (response.t3 - t4)) / 2;
                
                offsets.add(offset);
                delays.add(delay);
                
                // Wait briefly before next sample
                Thread.sleep(50);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        
        // Compute averages
        double avgOffset = offsets.stream().mapToLong(Long::longValue).average().orElse(0);
        double avgDelay = delays.stream().mapToLong(Long::longValue).average().orElse(0);
        
        return new double[] {avgOffset, avgDelay};
    }
}
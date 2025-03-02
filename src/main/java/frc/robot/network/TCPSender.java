package frc.robot.network;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

import com.google.gson.Gson;

/**
 * TCPSender is responsible for establishing a TCP connection to a remote device
 * (e.g., Raspberry Pi)
 * and sending configuration data in JSON format over the network.
 * 
 * @author Cameron Myhre
 * @since v2.0.0
 */
public class TCPSender {

    private Socket socket;
    private PrintWriter writer;
    private final Gson gson = new Gson(); // Gson instance for JSON serialization
    private final Object lock = new Object(); // Lock for thread-safe operations

    /**
     * Creates a new TCPSender object and establishes a TCP connection to the
     * specified Raspberry Pi.
     *
     * @param piIp   The IP address of the Raspberry Pi.
     * @param piPort The port number on which the Raspberry Pi is listening.
     * @throws IOException if an I/O error occurs when creating the socket or output
     *                     stream.
     */
    public TCPSender(String piIp, int piPort) throws IOException {
        this.socket = new Socket();
        // Establish a connection with a timeout of 5 seconds
        this.socket.connect(new InetSocketAddress(piIp, piPort), 5000);
        this.writer = new PrintWriter(
                new OutputStreamWriter(socket.getOutputStream(), StandardCharsets.UTF_8),
                true); // Auto-flush enabled
    }

    /**
     * Sends a configuration object to the connected Raspberry Pi.
     * The object is serialized to JSON before transmission.
     *
     * @param config The configuration object to send.
     * @throws IllegalStateException if the TCP connection is not open.
     */
    public void sendConfiguration(Object config) {
        synchronized (lock) {
            if (!isConnected()) {
                throw new IllegalStateException("TCP connection is not open. Unable to send data.");
            }

            // Serialize the configuration object to JSON and send
            String configJson = gson.toJson(config);
            writer.println(configJson);
        }
    }

    /**
     * Closes the TCP connection and releases associated resources.
     * This method ensures that the socket and writer are properly closed.
     */
    public void close() {
        synchronized (lock) {
            try {
                if (writer != null) {
                    writer.close();
                }
                if (socket != null && !socket.isClosed()) {
                    socket.close();
                }
            } catch (IOException e) {
                System.err.println("Error closing TCPSender resources: " + e.getMessage());
                e.printStackTrace();
            } finally {
                writer = null;
                socket = null;
            }
        }
    }

    /**
     * Attempts to reconnect to the specified IP/port. This method will close any
     * existing
     * connection, then create and connect a new socket, and recreate the
     * PrintWriter.
     *
     * @param piIp   The IP address of the Raspberry Pi.
     * @param piPort The port number on which the Raspberry Pi is listening.
     * @throws IOException if an I/O error occurs when creating the socket or output
     *                     stream.
     */
    public void reconnect(String piIp, int piPort) throws IOException {
        synchronized (lock) {
            // Close the old connection if it exists.
            // The close() method sets socket and writer to null.
            close();

            // Create a new socket
            this.socket = new Socket();
            // Attempt to connect to the Pi with a 5-second timeout (same as your
            // constructor)
            this.socket.connect(new InetSocketAddress(piIp, piPort), 5000);

            // Create a new PrintWriter with UTF-8
            this.writer = new PrintWriter(
                    new OutputStreamWriter(socket.getOutputStream(), StandardCharsets.UTF_8),
                    true);
        }
    }

    /**
     * Checks if the TCP connection is currently open.
     *
     * @return true if the connection is open, false otherwise.
     */
    public boolean isConnected() {
        synchronized (lock) {
            return socket != null && socket.isConnected() && !socket.isClosed();
        }
    }
}

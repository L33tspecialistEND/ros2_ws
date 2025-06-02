#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <modbus/modbus.h> // Required for Modbus RTU communication
#include <string>           // For std::string
#include <cstdint>          // For uint16_t

/**
 * @brief Represents an ultrasonic sensor communicating via Modbus RTU.
 *
 * This class provides an interface to connect to an ultrasonic sensor
 * over a serial Modbus RTU bus and read distance measurements.
 */
class UltrasonicSensor
{
public:
    /**
     * @brief Constructs an UltrasonicSensor object and initializes the Modbus context.
     *
     * This constructor sets up the necessary parameters for Modbus RTU communication
     * but does not establish a connection. Call `connect()` to open the serial port.
     *
     * @param port The serial port name (e.g., "/dev/ttyUSB0", "COM1").
     * @param baud_rate Baud rate for RTU communication (e.g., 9600, 19200).
     * @param parity Parity mode ('N' for None, 'E' for Even, 'O' for Odd).
     * @param data_bits Number of data bits (typically 8).
     * @param stop_bits Number of stop bits (typically 1).
     */
    UltrasonicSensor(const std::string& port, int baud_rate, char parity, int data_bits, int stop_bits);

    /**
     * @brief Destroys the UltrasonicSensor object and cleans up Modbus resources.
     *
     * This destructor disconnects from the Modbus bus if a connection
     * is active and frees the Modbus context.
     */
    ~UltrasonicSensor();

    /**
     * @brief Checks if the Modbus context has been initialized.
     *
     * This method indicates whether the internal Modbus context (`ctx_`) is valid
     * and ready for connection attempts.
     *
     * @return True if the Modbus context is initialized, false otherwise.
     */
    bool is_initialised();

    /**
     * @brief Connects to the Modbus RTU bus.
     *
     * Attempts to open the serial port and establish a connection to the Modbus bus.
     * This must be called before attempting to read data from sensors.
     *
     * @return True on successful connection, false on failure (e.g., port not found, permissions).
     */
    bool connect();

    /**
     * @brief Disconnects from the Modbus RTU bus.
     *
     * Closes the serial port and frees Modbus connection resources.
     */
    void disconnect();

    /**
     * @brief Reads the distance from a specific ultrasonic sensor.
     *
     * This function sends a Modbus read holding registers request to the specified
     * sensor and retrieves its distance measurement.
     *
     * @param slave_id The Modbus slave ID of the sensor (e.g., 1, 2, 3, or 4).
     * @param distance_mm A reference to a `uint16_t` variable where the
     * read distance in millimeters will be stored.
     * @return True on a successful read, false on failure.
     */
    bool read_distance(int slave_id, uint16_t& distance_mm);

private:
    std::string port_;         // The serial port name (e.g., "/dev/ttyUSB0").
    modbus_t* ctx_;            // Pointer to the Modbus context.
    bool is_connected_;        // Internal flag indicating if the Modbus bus is currently connected.

    /**
     * @brief Deleted copy constructor to prevent copying of UltrasonicSensor objects.
     *
     * Copying Modbus contexts can lead to resource management issues.
     */
    UltrasonicSensor(const UltrasonicSensor&) = delete;

    /**
     * @brief Deleted copy assignment operator to prevent assignment of UltrasonicSensor objects.
     *
     * Assigning Modbus contexts can lead to resource management issues.
     */
    UltrasonicSensor& operator=(const UltrasonicSensor&) = delete;
};

#endif // ULTRASONIC_SENSOR_H
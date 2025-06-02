#include "../include/ultrasonic_sensor/ultrasonic_sensor_control.h"
#include <iostream>
#include <cerrno>

UltrasonicSensor::UltrasonicSensor(const std::string& port, int baud_rate, char parity, int data_bits, int stop_bits)
    : port_(port), ctx_(nullptr), is_connected_(false)
{
    // Create a new modbus RTU context
    ctx_ = modbus_new_rtu(port_.c_str(), baud_rate, parity, data_bits, stop_bits);
    if (!is_initialised())
        std::cerr << "Error: Unable to create the libmodus context for port " << port_ << '\n';
}

UltrasonicSensor::~UltrasonicSensor()
{
    disconnect();   // Ensures the Modbus context is disconnected
    if (is_initialised())
    {
        modbus_free(ctx_);
        ctx_ = nullptr;
    }       
}

bool UltrasonicSensor::is_initialised()
{
    return !(ctx_ == nullptr);
}

bool UltrasonicSensor::connect()
{
    if (!is_initialised())
    {
        std::cerr << "Error: Modbus context not initialised. Cannot connect.\n";
        return false;
    }
    if (modbus_connect(ctx_) == -1)
    {
        std::cerr << "Error: Connection to " << port_ << " failed: " << modbus_strerror(errno) << '\n';
        return false;
    }
    std::cout << "Successfully connected to Modbus RTU on port " << port_ << '\n';
    is_connected_ = true;
    return true;
}

void UltrasonicSensor::disconnect()
{
    if (is_initialised())
    {
        modbus_close(ctx_);     // Close the modbus connection
        std::cout << "Disconnected from Modbus RTU on port " << port_ << '\n';
        is_connected_ = false;
    }
}

bool UltrasonicSensor::read_distance(int slave_id, uint16_t& distance_mm)
{
    if (!is_initialised())
    {
        std::cerr << "Error: Modbus context not initialised.\n";
        return false;
    }
    if (!is_connected_)
    {
        std::cerr << "Error: Not connected to Modbus bus. Call connect() first\n";
        return false;
    }
    if (modbus_set_slave(ctx_, slave_id) == -1)
    {
        std::cerr << "Error: Invalid slave ID " << slave_id << ": " << modbus_strerror(errno) << '\n';
        return false;
    }

    uint16_t received_distance;     // Holds distance read by the sensor
    int rc = modbus_read_registers(ctx_, 0x0100, 1, &received_distance);

    if (rc == -1)
    {
        std::cerr << "Error: Failed to read register from sensor " << slave_id << ": " << modbus_strerror(errno) << '\n';
        return false;
    }
    else if (rc == 0)
    {
        std::cerr << "Error: No response from sensor " << slave_id << '\n';
        return false;
    }
    else
    {
        distance_mm = received_distance;
        return true;
    }
}
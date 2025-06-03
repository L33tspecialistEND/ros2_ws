#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "../include/ultrasonic_sensor/ultrasonic_sensor_control.h"

using namespace std::chrono_literals;

class UltrasonicSensorDistancePub : public rclcpp::Node
{
public:
    UltrasonicSensorDistancePub()
    : Node("ultrasonic_sensor_distance_pub"),
      port_name_("/dev/ttyUSB0"),
      baud_rate_(9600),
      parity_('N'),
      data_bits_(8),
      stop_bits_(1),
      num_sensors_(2),  // Changed from 4 to 2 as I only have 2 sensors
      sensor_controller_(port_name_, baud_rate_, parity_, data_bits_, stop_bits_),
      is_sensor_connected_(false) // Initialize connection state
    {
        // Publishers
        front_sensor_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/distance/front", 10);
        rear_sensor_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/distance/rear", 10);
        left_sensor_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/distance/left", 10);
        right_sensor_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/distance/right", 10);

        if (!sensor_controller_.is_initialised()) {
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: Ultrasonic sensor controller failed to initialize. Shutting down node.");
            rclcpp::shutdown(); // Shutdown ROS 2 if context creation failed
            return;
        }

        if (!sensor_controller_.connect())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to ultrasonic sensor at startup. Node will attempt to reconnect.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Successfully connected to ultrasonic sensor on %s", port_name_.c_str());
            is_sensor_connected_ = true;
        }

        timer_ = this->create_wall_timer(300ms, std::bind(&UltrasonicSensorDistancePub::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Ultrasonic sensor distance publisher node started.");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr front_sensor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rear_sensor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_sensor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_sensor_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Member variables for sensor configuration
    const std::string port_name_;
    const int baud_rate_;
    const char parity_;
    const int data_bits_;
    const int stop_bits_;
    const int num_sensors_;

    UltrasonicSensor sensor_controller_;
    bool is_sensor_connected_; // Tracks current connection state

    void timer_callback()
    {
        if (!is_sensor_connected_)
        {
            if (!sensor_controller_.is_initialised()) {
                RCLCPP_FATAL(this->get_logger(), "CRITICAL: Sensor controller not initialized in timer_callback. Shutting down.");
                rclcpp::shutdown();
                return;
            }

            if (sensor_controller_.connect())
            {
                RCLCPP_INFO(this->get_logger(), "Re-established connection to ultrasonic sensor.");
                is_sensor_connected_ = true;
            } 
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Still unable to connect to ultrasonic sensor. Retrying...");
                return;
            }
        }

        // Only proceed with reading if connected
        if (is_sensor_connected_)
        {
            for (int iii = 0; iii < num_sensors_; ++iii)
            {
                uint16_t distance{};
                int slave_id = iii + 1; // slave ID starts from 1

                if (sensor_controller_.read_distance(slave_id, distance))
                {
                    std_msgs::msg::Float32 msg;
                    msg.data = static_cast<float>(distance);

                    switch (slave_id)
                    {
                        case 1:
                            front_sensor_publisher_->publish(msg);
                            RCLCPP_INFO(this->get_logger(), "Front sensor (ID 1): %.2f", msg.data);
                            break;
                        case 2:
                            rear_sensor_publisher_->publish(msg);
                            RCLCPP_INFO(this->get_logger(), "Rear sensor (ID 2): %.2f", msg.data);
                            break;
                        /*
                        case 3:
                            left_sensor_publisher_->publish(msg);
                            RCLCPP_INFO(this->get_logger(), "Left sensor (ID 3): %.2f", msg.data);
                            break;
                        case 4:
                            right_sensor_publisher_->publish(msg);
                            RCLCPP_INFO(this->get_logger(), "Right sensor (ID 4): %.2f", msg.data);
                            break;
                        */                        
                        default:
                            RCLCPP_WARN(this->get_logger(), "Unknown slave ID: %d", slave_id);
                            break;
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to read distance from sensor ID: %d. Marking sensor as potentially disconnected for next cycle if persistent.", slave_id);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UltrasonicSensorDistancePub>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
import rclpy
from rclpy.node import Node
from robot_led_interfaces.srv import SetRobotLedState
from .gpio_control import GPIOControl

class RobotLedService(Node):
    def __init__(self):
        super().__init__("robot_led_service_node")

        self.gpio_control = GPIOControl(mode = GPIOControl.NumberingMode.BOARD)

        self.srv = self.create_service(SetRobotLedState, "/set_robot_led_state", self.set_led_state_callback)

        self.get_logger().info("Robot LED Service Node ready.")

    

def main(args = None):
    rclpy.init(args = args)
    robot_led_service = RobotLedService()

    try:
        rclpy.spin(robot_led_service)
    except KeyboardInterrupt:
        robot_led_service.get_logger().info("Node interrupted by keyboard.")
    finally:
        robot_led_service.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import Jetson.GPIO as GPIO

class MaglockControl(Node):
    def __init__(self):
        super().__init__("maglock_control_service")
        self.pin = 13       # This is a GPIO pin on the Jetson
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.srv = self.create_service(SetBool, 'maglock', self.handle_request)
        self.get_logger().info('Maglock service ready (pin {})'.format(self.pin))

    def handle_request(self, request, response):
        if request.data:
            GPIO.output(self.pin, GPIO.LOW)
            self.get_logger().info("Unlocking maglock: (pin set LOW)")
        else:
            GPIO.output(self.pin, GPIO.HIGH)
            self.get_logger().info("Locking maglock: (pin set HIGH)")
        
        response.success = True
        response.message = "Maglock set to {}".format("UNLOCKED" if request.data else "LOCKED")
        
        return response
    
    def destroy_node(self):
        self.get_logger().info("Cleaning up GPIO...")
        GPIO.cleanup(self.pin)
        super().destroy_node()


def main(args = None):
    rclpy.init(args = args)
    node = MaglockControl()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
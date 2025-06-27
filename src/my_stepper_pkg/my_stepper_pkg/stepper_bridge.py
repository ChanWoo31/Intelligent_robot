#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import serial

class StepperBridge(Node):
    def __init__(self):
        super().__init__('stepper_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        self.sub = self.create_subscription(
            Point,
            'target_xy',
            self.on_target,
            10
        )
        self.get_logger().info('Stepper bridge ready on %s @%d' %
                               (self.ser.port, self.ser.baudrate))
    
    def on_target(self, msg: Point):
        x_mm = msg.x
        y_mm = msg.y
        cmd = f"{x_mm:.3f}, {y_mm:.3f}\n"
        self.ser.write(cmd.encode())
        self.get_logger().info(f"Sent to Arduino: {cmd.strip()}")
    
def main(args=None):
    rclpy.init(args=args)
    node = StepperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

    

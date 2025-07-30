#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32
import serial

class StepperBridge(Node):
    def __init__(self):
        super().__init__('stepper_bridge')
        self.ser = serial.Serial('/dev/ttyACM0', 1000000, timeout=0.1)
        self.target_sub = self.create_subscription(
            Point,
            'packing_position',
            self.on_target,
            10
        )
        self.arduino_done_pub = self.create_publisher(Bool, 'stepper_done', 10)
        self.ultra_pub = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        self.get_logger().info('Stepper bridge ready on %s @%d' %
                               (self.ser.port, self.ser.baudrate))
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line == "DONE":
                self.get_logger().info("Arduino x,y axis movement complete")
                self.arduino_done_pub.publish(Bool(data=True))
            else:
                self.get_logger().debug(f"Serial recv: {line}")
        
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line == "DONE":
                self.get_logger().info("Arduino x,y axis movement complete")
                self.arduino_done_pub.publish(Bool(data=True))
            elif line.startswith("DIST:"):
                try:
                    mm = float(line.split(":",1)[1])
                except ValueError:
                    continue
                self.ultra_pub.publish(Float32(data=mm))
                self.get_logger().debug(f"Ultrasonic: {mm:0f} mm")
            else:
                self.get_logger().debug(f"Serial recv: {line}")

    def on_target(self, msg: Point):
        x_mm = msg.x
        y_mm = msg.y
        cmd = f"{x_mm:.3f},{y_mm:.3f}\n"
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

    

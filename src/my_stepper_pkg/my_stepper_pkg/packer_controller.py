import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class PackerController(Node):
    def __init__(self):
        super().__init__('packer_controller')
        self.state = 'IDLE'
        self.stepper_done = False
        self.z_done = False
        self.place = None

        self.move_pub = self.create_publisher(Point, 'move_position', 10)
        self.vac_pub = self.create_publisher(Bool, 'vacuum_cmd', 10)

        self.create_subscription(Bool, 'stepper_done', self.cb_stepper, 10)
        self.create_subscription(Bool, 'z_done', self.cb_z, 10)
        self.create_subscription(Point, 'packing_position', self.cb_place, 10)

        self.timer = self.create_timer(0.1, self.run)

    def cb_place(self,msg: Point):
        if self.state == 'IDLE':
            self.get_logger().info(f'New place target <- {msg}')
            self.place = msg
            self.state = 'GO_PICKUP'

    def cb_stepper(self, msg: Bool):
        if not msg.data:
            return
        if self.state == 'WAIT_HOMING':
            self.get_logger().info('Homing & XY-0 Origin homed - ready')
            self.state = 'IDLE'
        else:
            self.get_logger().debug('-> stepper_done (move complete)')
            self.stepper_done = True

    def cb_z(self, msg: Bool):
        if msg.data:
            self.get_logger().debug('-> z_done')
            self.z_done = True

    def send_move(self, x, y, z):
        p = Point(x=x, y=y, z=z)
        self.move_pub.publish(p)
        self.get_logger().info(f'Move -> ({x:.1f}, {y:.1f}, {z:.1f})')
        self.stepper_done = False
        self.z_done = False

    def run(self):
        # 1) 픽업 위치로
        if self.state == 'GO_PICKUP':
            # 이 픽업위치는 내가 정해놓은거라 나중에 이거 다른 노드에서 전달해줘야함
            self.send_move(50.0, 50.0, 200.0)
            self.state = 'WAIT_PICKUP'

        # 2) 픽업 완료 -> 흡착 ON
        elif self.state == 'WAIT_PICKUP':
            if self.stepper_done and self.z_done:
                self.vac_pub.publish(Bool(data=True))
                self.get_logger().info('-> Vacuum ON')
                self.state = 'GO_PLACE'

        # 3) 플레이스 위치로
        elif self.state == 'GO_PLACE':
            if self.place is not None:
                self.send_move(self.place.x, self.place.y, self.place.z)
                self.state = 'WAIT_PLACE'

        # 4) 플레이스 완료 -> 흡착 OFF
        elif self.state == 'WAIT_PLACE':
            if self.stepper_done and self.z_done:
                self.vac_pub.publish(Bool(data=False))
                self.get_logger().info('-> Vacuum OFF')
                self.state = 'IDLE'
                self.place = None

def main(args=None):
    rclpy.init(args=args)
    node = PackerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

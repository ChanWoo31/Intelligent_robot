import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD

class ZControl(Node):
    def __init__(self):
        super().__init__('z_control')
        # 환경 설정
        self.DEVICENAME = '/dev/ttyUSB0'
        self.BAUDRATE = 1000000
        self.PROTOCOL = 1.0
        self.DXL_IDS = [1, 2]
        self.ADDR_TORQUE_ENABLE = 24
        self.ADDR_CW_ANGLE_LIMIT = 6
        self.ADDR_CCW_ANGLE_LIMIT = 8
        self.ADDR_MOVING_SPEED = 32
        self.TORQUE_ENABLE = 1

        self.port = PortHandler(self.DEVICENAME)
        self.port.openPort()
        self.port.setBaudRate(self.BAUDRATE)
        self.pkt = PacketHandler(self.PROTOCOL)

        for dxl_id in self.DXL_IDS:
            self.pkt.write2ByteTxRx(self.port, dxl_id,
                                    self.ADDR_CW_ANGLE_LIMIT, 0)
            self.pkt.write2ByteTxRx(self.port, dxl_id,
                                    self.ADDR_CCW_ANGLE_LIMIT, 0)
            self.pkt.write1ByteTxRx(self.port, dxl_id,
                                    self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        self.group_speed = GroupSyncWrite(
            self.port, self.pkt,
            self.ADDR_MOVING_SPEED, 2
        )

        self.target_z = None
        self.current_z = None
        self.kp = 20.0 # 튜닝 필요
        self.deadband = 20.0 # mm단위 허용 오차

        self.create_subscription(Point,   'packing_position',    self.cb_target, 10)
        self.create_subscription(Float32, 'ultrasonic_distance', self.cb_current,10)
        self.z_done_pub = self.create_publisher(Bool, 'z_done', 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    def cb_target(self, msg: Point):
        self.target_z = msg.z
        self.get_logger().info(f'목표 z={self.target_z:.1f} mm 설정')

    def cb_current(self, msg: Float32):
        self.current_z = 600 - 135 - msg.data

    def control_loop(self):
        if self.target_z is None or self.current_z is None:
            return

        error = self.target_z - self.current_z
        # 허용 오차 이내면 정지
        if abs(error) <= self.deadband:
            self._sync_write_speed(0)
            self.z_done_pub.publish(Bool(data=True))
            self.get_logger().info('z축 위치 도달, 모터 정지')
            self.target_z = None
            return

        # P 제어로 속도 계산 (음수일 땐 반대 방향 회전)
        base_speed = self.kp * error
        speed = int(max(min(self.kp * error, 1023), -1023))
        self._sync_write_speed(speed)
        self.get_logger().debug(f'error={error:.2f} cm, speed={speed}')

    def _sync_write_speed(self, speed: int):
        """
        두 모터에 속도명령을 동시에 전송합니다.
        모터1에는 speed, 모터2에는 -speed를 적용하여 항상 반대 방향으로 회전시킵니다.
        Wheel 모드를 위해 bit10(0x400)로 방향을 지정합니다.
        """
        self.group_speed.clearParam()
        for idx, dxl_id in enumerate(self.DXL_IDS):
            raw_speed = speed if idx == 0 else -speed
            # 10비트 속도 값 추출
            abs_speed = abs(int(raw_speed)) & 0x3FF
            # 음수일 경우 CCW 방향 비트 추가
            if raw_speed < 0:
                abs_speed |= 0x400
            # 리틀 엔디언 파라미터 생성
            param = [(abs_speed & 0xFF), (abs_speed >> 8) & 0xFF]
            self.group_speed.addParam(dxl_id, bytearray(param))
        self.group_speed.txPacket()

    def destroy_node(self):
        # 노드 종료 시 모터 토크 끄기
        for dxl_id in self.DXL_IDS:
            self.pkt.write1ByteTxRx(self.port, dxl_id,
                self.ADDR_TORQUE_ENABLE, 0)
        self.port.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()

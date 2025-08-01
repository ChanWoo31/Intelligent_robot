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
        self.kp = 10.0 # 튜닝 필요
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
        # 목표 및 현재값 준비 여부 확인
        if self.target_z is None or self.current_z is None:
            return

        # 오차 계산
        error = self.target_z - self.current_z
        self.get_logger().debug(f'[DBG] target={self.target_z:.1f}, current={self.current_z:.1f}, error={error:.1f}')

        # 허용 오차 이내 도달 시 모터 정지 후 완료 퍼블리시
        if abs(error) <= self.deadband:
            self._sync_write_speed(0, 0)
            self.z_done_pub.publish(Bool(data=True))
            self.get_logger().info('Z axis reached target, motor stopped')
            self.target_z = None
            return

        # P 제어로 속도 크기 계산
        base_speed = self.kp * abs(error)
        speed_val = int(max(min(base_speed, 1023), 0))

        # 방향에 따라 모터1과 모터2 속도 설정
        if error < 0:
            # 목표 < 현재: 내려가야 함
            s1 = speed_val    # motor1 forward
            s2 = -speed_val   # motor2 reverse
        else:
            # 목표 > 현재: 올라가야 함
            s1 = -speed_val   # motor1 reverse
            s2 = speed_val    # motor2 forward

        # 동기 제어
        self._sync_write_speed(s1, s2)
        self.get_logger().debug(f'Applying speeds -> m1: {s1}, m2: {s2}')

    def _sync_write_speed(self, speed1: int, speed2: int):
        """
        두 모터에 Wheel 모드 속도명령 전송:
          * speed1: 첫 번째 모터(DXL_IDS[0])
          * speed2: 두 번째 모터(DXL_IDS[1])
        음수일 때 bit10(0x400)로 CCW 방향 지정
        """
        self.group_speed.clearParam()
        for dxl_id, raw_speed in zip(self.DXL_IDS, [speed1, speed2]):
            val = abs(raw_speed) & 0x3FF
            if raw_speed < 0:
                val |= 0x400
            param = [val & 0xFF, (val >> 8) & 0xFF]
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

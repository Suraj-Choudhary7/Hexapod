#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from dynamixel_sdk import (PortHandler, PacketHandler, GroupSyncWrite,DXL_LOBYTE, DXL_HIBYTE, COMM_SUCCESS,)

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
LEN_GOAL_POSITION  = 2
PROTOCOL_VERSION   = 1.0
RAD_TO_RAW = 1023.0 / (300.0 * 3.141592653589793 / 180.0) 
CENTER_RAW = 512

COXA_OFFSET  =  0.000
FEMUR_OFFSET = -0.8264
TIBIA_OFFSET = -1.7349

JOINT_CONFIG = {
    'FH_LH_1': {'id': 11, 'offset': COXA_OFFSET,  'dir':  1},
    'FH_LH_2': {'id': 12, 'offset': FEMUR_OFFSET, 'dir': -1},
    'FH_LH_3': {'id': 13, 'offset': TIBIA_OFFSET, 'dir': -1},

    'FH_RH_1': {'id': 61, 'offset': COXA_OFFSET,  'dir':  1},
    'FH_RH_2': {'id': 62, 'offset': FEMUR_OFFSET, 'dir': -1},
    'FH_RH_3': {'id': 63, 'offset': TIBIA_OFFSET, 'dir': -1},

    'MH_LH_1': {'id': 21, 'offset': COXA_OFFSET,  'dir':  1},
    'MH_LH_2': {'id': 22, 'offset': FEMUR_OFFSET, 'dir': -1},
    'MH_LH_3': {'id': 23, 'offset': TIBIA_OFFSET, 'dir': -1},

    'MH_RH_1': {'id': 51, 'offset': COXA_OFFSET,  'dir':  1},
    'MH_RH_2': {'id': 52, 'offset': FEMUR_OFFSET, 'dir': -1},
    'MH_RH_3': {'id': 53, 'offset': TIBIA_OFFSET, 'dir': -1},

    'BH_RH_1': {'id': 41, 'offset': COXA_OFFSET,  'dir':  1},
    'BH_RH_2': {'id': 42, 'offset': FEMUR_OFFSET, 'dir': -1},
    'BH_RH_3': {'id': 43, 'offset': TIBIA_OFFSET, 'dir': -1},

    'BH_LH_1': {'id': 31, 'offset': COXA_OFFSET,  'dir':  1},
    'BH_LH_2': {'id': 32, 'offset': FEMUR_OFFSET, 'dir': -1},
    'BH_LH_3': {'id': 33, 'offset': TIBIA_OFFSET, 'dir': -1},
}

JOINT_ORDER = [
    'FH_LH_1', 'FH_LH_2', 'FH_LH_3',
    'FH_RH_1', 'FH_RH_2', 'FH_RH_3',
    'MH_LH_1', 'MH_LH_2', 'MH_LH_3',
    'MH_RH_1', 'MH_RH_2', 'MH_RH_3',
    'BH_RH_1', 'BH_RH_2', 'BH_RH_3',
    'BH_LH_1', 'BH_LH_2', 'BH_LH_3',
]

def rad_to_raw(angle_rad, offset_rad, direction):
    raw = CENTER_RAW + direction * (angle_rad + offset_rad) * RAD_TO_RAW
    return int(max(0, min(1023, round(raw))))

class DynamixelDriverNode(Node):
    def __init__(self):
        super().__init__('dynamixel_driver')

        self.declare_parameter('device',   '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)

        device   = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value

        self._port   = PortHandler(device)
        self._packet = PacketHandler(PROTOCOL_VERSION)

        if not self._port.openPort():
            self.get_logger().fatal(f'Cannot open port: {device}')
            raise RuntimeError(f'Cannot open port: {device}')

        if not self._port.setBaudRate(baudrate):
            self.get_logger().fatal(f'Cannot set baudrate: {baudrate}')
            raise RuntimeError(f'Cannot set baudrate: {baudrate}')

        self.get_logger().info(f'Opened {device} at {baudrate} baud.')
        self._sync = GroupSyncWrite(self._port, self._packet,ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        self._set_torque(True)
        self._sub = self.create_subscription(JointTrajectory,'/leg_trajectory_controller/joint_trajectory',self._callback,10)
        self.get_logger().info('Dynamixel driver ready.')

    def _callback(self, msg: JointTrajectory):
        if not msg.points:
            return

        pos_map = dict(zip(msg.joint_names, msg.points[0].positions))

        for name in JOINT_ORDER:
            if name not in pos_map:
                self.get_logger().warn(f'Missing joint in message: {name}', throttle_duration_sec=5.0)
                self._sync.clearParam()
                return

            cfg = JOINT_CONFIG[name]
            raw = rad_to_raw(pos_map[name], cfg['offset'], cfg['dir'])
            ok  = self._sync.addParam(cfg['id'], [DXL_LOBYTE(raw), DXL_HIBYTE(raw)])
            if not ok:
                self.get_logger().error(f'addParam failed for {name} (ID {cfg["id"]})')
                self._sync.clearParam()
                return

        result = self._sync.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(f'SyncWrite failed: {self._packet.getTxRxResult(result)}')

        self._sync.clearParam()

    def _set_torque(self, enable: bool):
        val = 1 if enable else 0
        self.get_logger().info(f'{"Enabling" if enable else "Disabling"} torque...')
        for name, cfg in JOINT_CONFIG.items():
            res, _ = self._packet.write1ByteTxRx(self._port, cfg['id'], ADDR_TORQUE_ENABLE, val)
            if res != COMM_SUCCESS:
                self.get_logger().warn(f'Torque set failed for {name} (ID {cfg["id"]})')

    def destroy_node(self):
        self.get_logger().info('Shutting down — disabling torque and closing port.')
        self._set_torque(False)
        self._port.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = DynamixelDriverNode()
        rclpy.spin(node)
    except RuntimeError:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
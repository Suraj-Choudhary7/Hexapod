#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupSyncWrite,
    DXL_LOBYTE, DXL_HIBYTE, COMM_SUCCESS,
)

# AX-12A Control Table
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
LEN_GOAL_POSITION  = 2

PROTOCOL_VERSION   = 1.0

# Conversion: 1023 ticks over 300 degrees
RAD_TO_RAW = 1023.0 / (300.0 * 3.141592653589793 / 180.0)  # ≈ 195.38
CENTER_RAW = 512

# Physical neutral positions from your manual test:
#   coxa  → 512 raw  → offset = (512 - 512) / 195.38 = 0.000 rad
#   femur → 631 raw  → offset = (631 - 512) / 195.38 = 0.609 rad
#   tibia → 716 raw  → offset = (716 - 512) / 195.38 = 1.044 rad
COXA_OFFSET  =  0.000
FEMUR_OFFSET = -0.8264
TIBIA_OFFSET = -1.7349

# Joint config: maps ROS joint name → Dynamixel ID, offset, direction
# direction = -1 for right-side coxa joints (mirrored legs)
JOINT_CONFIG = {
    # Leg 1 — Front Left (FH_LH)
    'FH_LH_1': {'id': 11, 'offset': COXA_OFFSET,  'dir':  1},
    'FH_LH_2': {'id': 12, 'offset': FEMUR_OFFSET, 'dir': -1},
    'FH_LH_3': {'id': 13, 'offset': TIBIA_OFFSET, 'dir': -1},

    # Leg 2 — Front Right (FH_RH)
    'FH_RH_1': {'id': 61, 'offset': COXA_OFFSET,  'dir':  1},
    'FH_RH_2': {'id': 62, 'offset': FEMUR_OFFSET, 'dir': -1},
    'FH_RH_3': {'id': 63, 'offset': TIBIA_OFFSET, 'dir': -1},

    # Leg 3 — Middle Left (MH_LH)
    'MH_LH_1': {'id': 21, 'offset': COXA_OFFSET,  'dir':  1},
    'MH_LH_2': {'id': 22, 'offset': FEMUR_OFFSET, 'dir': -1},
    'MH_LH_3': {'id': 23, 'offset': TIBIA_OFFSET, 'dir': -1},

    # Leg 4 — Middle Right (MH_RH)
    'MH_RH_1': {'id': 51, 'offset': COXA_OFFSET,  'dir':  1},
    'MH_RH_2': {'id': 52, 'offset': FEMUR_OFFSET, 'dir': -1},
    'MH_RH_3': {'id': 53, 'offset': TIBIA_OFFSET, 'dir': -1},

    # Leg 5 — Back Right (BH_RH)
    'BH_RH_1': {'id': 41, 'offset': COXA_OFFSET,  'dir':  1},
    'BH_RH_2': {'id': 42, 'offset': FEMUR_OFFSET, 'dir': -1},
    'BH_RH_3': {'id': 43, 'offset': TIBIA_OFFSET, 'dir': -1},

    # Leg 6 — Back Left (BH_LH)
    'BH_LH_1': {'id': 31, 'offset': COXA_OFFSET,  'dir':  1},
    'BH_LH_2': {'id': 32, 'offset': FEMUR_OFFSET, 'dir': -1},
    'BH_LH_3': {'id': 33, 'offset': TIBIA_OFFSET, 'dir': -1},
}

# Same order as hexa_node's joint_names list
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

        # Open port
        self._port   = PortHandler(device)
        self._packet = PacketHandler(PROTOCOL_VERSION)

        if not self._port.openPort():
            self.get_logger().fatal(f'Cannot open port: {device}')
            raise RuntimeError(f'Cannot open port: {device}')

        if not self._port.setBaudRate(baudrate):
            self.get_logger().fatal(f'Cannot set baudrate: {baudrate}')
            raise RuntimeError(f'Cannot set baudrate: {baudrate}')

        self.get_logger().info(f'Opened {device} at {baudrate} baud.')

        # SyncWrite group — same as your test script
        self._sync = GroupSyncWrite(
            self._port, self._packet,
            ADDR_GOAL_POSITION, LEN_GOAL_POSITION
        )

        # Enable torque on all 18 motors
        self._set_torque(True)

        # Subscribe to the same topic hexa_node publishes to
        self._sub = self.create_subscription(
            JointTrajectory,
            '/leg_trajectory_controller/joint_trajectory',
            self._callback,
            10
        )

        self.get_logger().info('Dynamixel driver ready.')

    def _callback(self, msg: JointTrajectory):
        if not msg.points:
            return

        # Map name → angle from the incoming message
        pos_map = dict(zip(msg.joint_names, msg.points[0].positions))

        # Load all 18 joints into SyncWrite
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

        # Fire one SyncWrite packet — moves all 18 motors simultaneously
        result = self._sync.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(f'SyncWrite failed: {self._packet.getTxRxResult(result)}')

        self._sync.clearParam()

    def _set_torque(self, enable: bool):
        val = 1 if enable else 0
        self.get_logger().info(f'{"Enabling" if enable else "Disabling"} torque...')
        for name, cfg in JOINT_CONFIG.items():
            res, _ = self._packet.write1ByteTxRx(
                self._port, cfg['id'], ADDR_TORQUE_ENABLE, val
            )
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
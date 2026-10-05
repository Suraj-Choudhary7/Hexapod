#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

COXA_OFFSET  =  0.000
FEMUR_OFFSET = -0.8264
TIBIA_OFFSET_SIM = -0.5

JOINT_CONFIG = {
    'FH_LH_1': {'offset': COXA_OFFSET,    'dir':  1},
    'FH_LH_2': {'offset': FEMUR_OFFSET,   'dir': -1},
    'FH_LH_3': {'offset': TIBIA_OFFSET_SIM, 'dir':  1},
    'FH_RH_1': {'offset': COXA_OFFSET,    'dir':  1},
    'FH_RH_2': {'offset': FEMUR_OFFSET,   'dir': -1},
    'FH_RH_3': {'offset': TIBIA_OFFSET_SIM, 'dir':  1},
    'MH_LH_1': {'offset': COXA_OFFSET,    'dir':  1},
    'MH_LH_2': {'offset': FEMUR_OFFSET,   'dir': -1},
    'MH_LH_3': {'offset': TIBIA_OFFSET_SIM, 'dir':  1},
    'MH_RH_1': {'offset': COXA_OFFSET,    'dir':  1},
    'MH_RH_2': {'offset': FEMUR_OFFSET,   'dir': -1},
    'MH_RH_3': {'offset': TIBIA_OFFSET_SIM, 'dir':  1},
    'BH_RH_1': {'offset': COXA_OFFSET,    'dir':  1},
    'BH_RH_2': {'offset': FEMUR_OFFSET,   'dir': -1},
    'BH_RH_3': {'offset': TIBIA_OFFSET_SIM, 'dir':  1},
    'BH_LH_1': {'offset': COXA_OFFSET,    'dir':  1},
    'BH_LH_2': {'offset': FEMUR_OFFSET,   'dir': -1},
    'BH_LH_3': {'offset': TIBIA_OFFSET_SIM, 'dir':  1},
}

class SimRelayNode(Node):
    def __init__(self):
        super().__init__('sim_relay')

        self._sub = self.create_subscription(JointTrajectory,'/hexa_node/joint_trajectory',self._callback,10,)
        self._pub = self.create_publisher(JointTrajectory,'/leg_trajectory_controller/joint_trajectory',10,)
        self.get_logger().info('sim_relay started — applying hardware offsets + direction corrections.')

    def _callback(self, msg: JointTrajectory):
        for point in msg.points:
            positions = list(point.positions)
            for i, name in enumerate(msg.joint_names):
                if name in JOINT_CONFIG:
                    cfg = JOINT_CONFIG[name]
                    positions[i] = cfg['dir'] * (positions[i] + cfg['offset'])
            point.positions = positions
        self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
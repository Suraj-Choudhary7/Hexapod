#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from hexa_control.kinematics import Kinematics
from hexa_control.gait import CpgGait

class HexapodControllerNode(Node):
    def __init__(self):
        super().__init__('hexa_node')

        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('gait_frequency', 0.5)
        self.declare_parameter('step_height', 0.06)
        self.declare_parameter('stance_reach_x', 0.160)
        self.declare_parameter('standing_height', -0.190)
        self.declare_parameter('transition_duration_sec', 1.5)
        self.declare_parameter('idle_to_rest_sec', 5.0)

        rate_hz = self.get_parameter('control_rate_hz').value
        gait_freq = self.get_parameter('gait_frequency').value
        step_h = self.get_parameter('step_height').value
        stance_x = self.get_parameter('stance_reach_x').value
        stand_z = self.get_parameter('standing_height').value
        self.transition_duration = self.get_parameter('transition_duration_sec').value
        self.idle_to_rest_sec = self.get_parameter('idle_to_rest_sec').value
        self.dt = 1.0 / rate_hz

        self.kinematics = Kinematics(l1=0.051, l2=0.110, l3=0.148)
        self.gait = CpgGait(frequency=gait_freq, amplitude=step_h, normal_x=stance_x, normal_z=stand_z)

        self.joint_names = [
            'FH_LH_1', 'FH_LH_2', 'FH_LH_3',
            'FH_RH_1', 'FH_RH_2', 'FH_RH_3',
            'MH_LH_1', 'MH_LH_2', 'MH_LH_3',
            'MH_RH_1', 'MH_RH_2', 'MH_RH_3',
            'BH_RH_1', 'BH_RH_2', 'BH_RH_3',
            'BH_LH_1', 'BH_LH_2', 'BH_LH_3'
        ]
        self.legs = ['FH_LH', 'FH_RH', 'MH_LH', 'MH_RH', 'BH_RH', 'BH_LH']

        stand_angles = self.kinematics.inverse_kinematics(stance_x, 0.0, stand_z)
        self.STAND_POSE = {}
        for leg in self.legs:
            for i, ang in enumerate(stand_angles, start=1):
                self.STAND_POSE[f'{leg}_{i}'] = ang

        rest_angles = self.kinematics.inverse_kinematics(0.09, 0.0, stand_z)
        self.REST_POSE = {}
        for leg in self.legs:
            for i, ang in enumerate(rest_angles, start=1):
                self.REST_POSE[f'{leg}_{i}'] = ang

        self.state = 'RESTING'
        self.current_angles = dict(self.REST_POSE)   

        self._transition_start_pose = None   
        self._transition_target = None
        self._transition_steps_total = 0
        self._transition_step_count = 0
        self._post_transition_state = None  

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        self._idle_time = 0.0

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.traj_pub = self.create_publisher(JointTrajectory, '/leg_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Hexapod CPG Controller started (RESTING).')

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z

    def _publish_angles(self, angles_dict):
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [angles_dict[name] for name in self.joint_names]
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        traj_msg.points.append(point)

        self.traj_pub.publish(traj_msg)
        self.current_angles = dict(angles_dict)

    def _start_transition(self, target_pose, post_state):
        self.state = 'TRANSITIONING'
        self._transition_start_pose = dict(self.current_angles)  
        self._transition_target = target_pose
        self._transition_steps_total = max(1, int(self.transition_duration / self.dt))
        self._transition_step_count = 0
        self._post_transition_state = post_state

    def _step_transition(self):
        self._transition_step_count += 1
        alpha = self._transition_step_count / float(self._transition_steps_total)
        alpha = min(1.0, alpha)

        interp = {
            name: self._transition_start_pose[name]
                  + alpha * (self._transition_target[name] - self._transition_start_pose[name])
            for name in self.joint_names
        }
        self._publish_angles(interp)

        if alpha >= 1.0:
            self.state = self._post_transition_state
            self.get_logger().info(f'Transition complete -> {self.state}')

    def control_loop(self):
        is_moving = (abs(self.cmd_vx) > 0.01 or abs(self.cmd_vy) > 0.01 or abs(self.cmd_wz) > 0.01)

        if self.state == 'TRANSITIONING':
            self._step_transition()
            return

        if is_moving and self.state == 'RESTING':
            self.get_logger().info('Movement command received — auto standing up...')
            self._idle_time = 0.0
            self._start_transition(self.STAND_POSE, post_state='WALKING')
            return

        if is_moving or self.state == 'WALKING':
            self.state = 'WALKING' if is_moving else 'STANDING'

            if is_moving:
                self._idle_time = 0.0
            else:
                self._idle_time += self.dt
                if self._idle_time >= self.idle_to_rest_sec:
                    self.get_logger().info(
                        f'Idle for {self.idle_to_rest_sec:.0f}s — returning to REST...'
                    )
                    self._idle_time = 0.0
                    self._start_transition(self.REST_POSE, post_state='RESTING')
                    return

            vel_magnitude = math.sqrt(self.cmd_vx**2 + self.cmd_vy**2 + self.cmd_wz**2)
            self.gait.cpg_phase(self.dt, vel_magnitude)

            joint_positions_dict = {}
            for leg in self.legs:
                x, y, z = self.gait.foot_trajectory(leg, self.cmd_vx, self.cmd_vy, self.cmd_wz)
                q1, q2, q3 = self.kinematics.inverse_kinematics(x, y, z)
                joint_positions_dict[f'{leg}_1'] = q1
                joint_positions_dict[f'{leg}_2'] = q2
                joint_positions_dict[f'{leg}_3'] = q3

            self._publish_angles(joint_positions_dict)
            return

    def destroy_node(self):
        """On shutdown, synchronously move all legs to REST_POSE before closing."""
        self.get_logger().info('Shutdown requested — moving to REST pose...')

        self.timer.cancel()

        steps = max(1, int(self.transition_duration / self.dt))
        start_pose = dict(self.current_angles)

        for step in range(1, steps + 1):
            alpha = min(1.0, step / float(steps))
            interp = {
                name: start_pose[name]
                      + alpha * (self.REST_POSE[name] - start_pose[name])
                for name in self.joint_names
            }
            self._publish_angles(interp)
            time.sleep(self.dt)

        self.get_logger().info('REST pose reached — shutting down.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HexapodControllerNode()
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
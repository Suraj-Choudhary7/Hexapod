#!/usr/bin/env python3
"""
teleop_keyboard.py — Custom keyboard teleop for hexapod.
Supports linear x, linear y, angular z and their combinations.

Key layout:
         w
    a    s    d        — forward / strafe-left / back / strafe-right
         x

    q    e             — forward-left / forward-right diagonal
    z    c             — back-left    / back-right   diagonal

    j         l        — turn left / turn right
         k             — stop

    +/-               — increase / decrease speed
    Space             — emergency stop (zero all)
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# --- Key bindings: (linear.x, linear.y, angular.z) ---
KEY_BINDINGS = {
    # Straight moves
    'w': ( 1.0,  0.0,  0.0),   # forward
    's': (-1.0,  0.0,  0.0),   # backward
    'a': ( 0.0,  1.0,  0.0),   # strafe left
    'd': ( 0.0, -1.0,  0.0),   # strafe right

    # Diagonals
    'q': ( 1.0,  1.0,  0.0),   # forward-left
    'e': ( 1.0, -1.0,  0.0),   # forward-right
    'z': (-1.0,  1.0,  0.0),   # back-left
    'c': (-1.0, -1.0,  0.0),   # back-right

    # Turns
    'j': ( 0.0,  0.0,  1.0),   # turn left
    'l': ( 0.0,  0.0, -1.0),   # turn right

    # Forward + turn combinations
    'u': ( 1.0,  0.0,  1.0),   # forward + turn left
    'o': ( 1.0,  0.0, -1.0),   # forward + turn right
}

STOP_KEY   = 'k'
SPEED_UP   = '+'
SPEED_DOWN = '-'
ESTOP      = ' '

HELP_MSG = """
Hexapod Keyboard Teleop
-----------------------
Movement:
   q    w    e
   a    s    d      (s = stop movement, k = full stop)
   z         c

   w : forward          s : backward
   a : strafe left      d : strafe right
   q : fwd + left       e : fwd + right
   z : back + left      c : back + right

Turning:
   j : turn left        l : turn right
   u : fwd + turn left  o : fwd + turn right

Speed:
   + : increase speed
   - : decrease speed

k / Space : STOP
Ctrl+C    : quit
-----------------------
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        self.declare_parameter('linear_speed',  0.10)
        self.declare_parameter('angular_speed', 0.50)
        self.declare_parameter('speed_step',    0.02)

        self.linear_speed  = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.speed_step    = self.get_parameter('speed_step').value

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        print(HELP_MSG)
        print(f'Linear speed : {self.linear_speed:.2f} m/s')
        print(f'Angular speed: {self.angular_speed:.2f} rad/s')
        print()

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = get_key(settings)

                if key == '\x03':   # Ctrl+C
                    break

                msg = Twist()

                if key in KEY_BINDINGS:
                    vx, vy, wz = KEY_BINDINGS[key]
                    msg.linear.x  = vx * self.linear_speed
                    msg.linear.y  = vy * self.linear_speed
                    msg.angular.z = wz * self.angular_speed
                    self._pub.publish(msg)
                    print(f'  [{key}] vx={msg.linear.x:+.2f}  vy={msg.linear.y:+.2f}  wz={msg.angular.z:+.2f}',
                          end='\r')

                elif key in (STOP_KEY, ESTOP):
                    self._pub.publish(msg)   # all zeros
                    print(f'  STOP                                    ', end='\r')

                elif key == SPEED_UP:
                    self.linear_speed  = round(self.linear_speed  + self.speed_step, 3)
                    self.angular_speed = round(self.angular_speed + self.speed_step, 3)
                    print(f'\n  Speed up → linear={self.linear_speed:.2f}  angular={self.angular_speed:.2f}')

                elif key == SPEED_DOWN:
                    self.linear_speed  = max(0.01, round(self.linear_speed  - self.speed_step, 3))
                    self.angular_speed = max(0.01, round(self.angular_speed - self.speed_step, 3))
                    print(f'\n  Speed down → linear={self.linear_speed:.2f}  angular={self.angular_speed:.2f}')

        finally:
            # Restore terminal and send stop
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self._pub.publish(Twist())
            print('\nStopped.')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
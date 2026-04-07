# Copyright 2025 Lihan Chen
# Modified based on teleop_twist_keyboard from ROS2
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import threading

import rclpy
from pb_rm_interfaces.msg import GimbalCmd
from rclpy.node import Node
from std_msgs.msg import Header

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty

msg = """
Reading from the keyboard and Publishing to GimbalCmd `cmd_gimbal`!
---------------------------
移动控制:
   q    w    e
   a    s    d
   z    x    c

w/x : 向上/向下 gimbal_pitch
a/d : 向左/向右 gimbal_yaw
q/z : 增加/减少 pitch 步长 10%
e/c : 增加/减少 yaw 步长 10%

Ctrl+C 退出
"""

move_bindings = {
    "w": ("pitch", -1),
    "x": ("pitch", 1),
    "a": ("yaw", 1),
    "d": ("yaw", -1),
}

step_bindings = {
    "q": ("pitch_step", 1.1),
    "z": ("pitch_step", 0.9),
    "e": ("yaw_step", 1.1),
    "c": ("yaw_step", 0.9),
}


def get_key(settings):
    if sys.platform == "win32":
        return msvcrt.getwch()
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    if sys.platform == "win32" or not sys.stdin.isatty():
        return None
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    if sys.platform == "win32" or not old_settings:
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class KeyboardControl(Node):
    def __init__(self):
        super().__init__("keyboard_control")
        self.publisher = self.create_publisher(GimbalCmd, "cmd_gimbal", 10)

        self.gimbal_pitch = 0.0
        self.gimbal_yaw = 0.0
        self.pitch_step = 0.1
        self.yaw_step = 0.1

    def publish_state(self):
        msg = GimbalCmd()
        msg.header = Header(stamp=self.get_clock().now().to_msg())

        msg.yaw_type = GimbalCmd.ABSOLUTE_ANGLE
        msg.pitch_type = GimbalCmd.ABSOLUTE_ANGLE

        msg.position.pitch = float(self.gimbal_pitch)
        msg.position.yaw = float(self.gimbal_yaw)

        self.publisher.publish(msg)

    def print_status(self):
        os.system("clear")
        print(msg)
        print(
            f"current_pitch: {self.gimbal_pitch:.2f} rad, current_yaw: {self.gimbal_yaw:.2f} rad\n"
            f"pitch_step: {self.pitch_step:.2f}, yaw_step: {self.yaw_step:.2f}"
        )


def main():
    settings = save_terminal_settings()

    rclpy.init()
    node = KeyboardControl()
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.daemon = True
    spinner.start()

    try:
        node.print_status()
        while rclpy.ok():
            key = get_key(settings)
            if key in move_bindings:
                joint, direction = move_bindings[key]
                if joint == "pitch":
                    node.gimbal_pitch += direction * node.pitch_step
                elif joint == "yaw":
                    node.gimbal_yaw += direction * node.yaw_step
            elif key in step_bindings:
                param, factor = step_bindings[key]
                if param == "pitch_step":
                    node.pitch_step = max(0.01, node.pitch_step * factor)
                elif param == "yaw_step":
                    node.yaw_step = max(0.01, node.yaw_step * factor)
            elif key == "\x03":  # Ctrl+C
                break
            else:
                continue

            node.publish_state()
            node.print_status()

    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
    finally:
        restore_terminal_settings(settings)
        node.destroy_node()
        rclpy.shutdown()
        spinner.join()


if __name__ == "__main__":
    main()

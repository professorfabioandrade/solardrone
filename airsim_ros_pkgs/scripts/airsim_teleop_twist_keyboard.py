#!/usr/bin/env python3

import threading

import geometry_msgs.msg
from airsim_interfaces.msg import VelCmd
from airsim_interfaces.srv import *

import sys, rclpy, termios, tty
from typing import List

from airsim_ros_pkgs.base_teleop_node import BaseTeleopNode

class TeleopKeyboard(BaseTeleopNode):
    def __init__(self) -> None:
        super.__init__("teleop_twist_keyboard")

        settings = self.saveTerminalSettings()

        ## initial variables
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0.0

        ## parameters
        stamped = node.declare_parameter('stamped', False).value
        frame_id = node.declare_parameter('frame_id', '').value
        if not stamped and frame_id:
            raise Exception("'frame_id' can only be set when 'stamped' is True")

        self.pub = self.create_publisher(VelCmd, 'airsim_node/Drone_1/vel_cmd_body_frame', 1)

    def __call__(self) -> None:
        twist_msg = VelCmd() 
        try:
            print(self.msg())
            print(self.vels(self.speed, self.turn))
            while True:
                key = self.getKey(settings)
                if key in self.moveBindings.keys():
                    self.x = self.moveBindings[key][0]
                    self.y = self.moveBindings[key][1]
                    self.z = self.moveBindings[key][2]
                    self.th = self.moveBindings[key][3]
                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]

                    print(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        print(self.msg())
                    self.status = (self.status + 1) % 15
                elif key == '1':
                    setTakeoffMode()
                elif key == '2':
                    setLandMode()
                else:
                    self.x = 0.0
                    self.y = 0.0
                    self.z = 0.0
                    self.th = 0.0
                    if (key == '\x03'):
                        break

                twist_msg.twist.linear.x = self.x * self.speed
                twist_msg.twist.linear.y = self.y * self.speed
                twist_msg.twist.linear.z = self.z * self.speed
                twist_msg.twist.angular.x = 0.0
                twist_msg.twist.angular.y = 0.0
                twist_msg.twist.angular.z = self.th * self.turn
                self.pub.publish(twist_msg)

        except Exception as e:
            print(e)

        finally:
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = 0.0
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
            self.pub.publish(twist_msg)
            rclpy.shutdown()
            spinner.join()

            restoreTerminalSettings(settings)

    @staticmethod
    def saveTerminalSettings():
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

    @staticmethod
    def restoreTerminalSettings(old_settings: List) -> None:
        if sys.platform == 'win32':
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    @staticmethod 
    def vels(speed: float, turn: float) -> str:
        return f'currently:\tspeed {speed}\tturn {turn} '

    @staticmethod
    def getKey(settings: List) -> str:
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def setLandMode():
        landService = self.create_client(Land, '/airsim_node/Drone_1/land')
        while not landService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            req = Land.Request()
            req.wait_on_last_task = True
            resp = landService.call_async(req)
            rclpy.spin_until_future_complete(self, resp)
        except Exception as e:
            self.get_logger().info("service land call failed: %s. The vehicle cannot land "%e)

    def setTakeoffMode():
        takeoffService = self.create_client(Takeoff, '/airsim_node/Drone_1/takeoff')
        while not takeoffService.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        try:
            req = Takeoff.Request()
            req.wait_on_last_task = True
            resp = takeoffService.call_async(req)
            rclpy.spin_until_future_complete(self, resp)
        except Exception as e:
            self.get_logger().info("service takeoff call failed: %s. The vehicle cannot takeoff "%e)


def main():
    rclpy.init()
    teleop = TeleopKeyboard()

    try:
        while rclpy.ok():
            teleop()
    except KeyboardInterrupt:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
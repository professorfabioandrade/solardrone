#!/usr/bin/env python3

from rclpy.node import Node

class BaseTeleopNode(Node):
    def __init__(self, node_name: str = "teleop") -> None:
        super().__init__(node_name)

        self.moveBindings = {
            'i': (1, 0, 0, 0),
            'o': (1, 0, 0, 1),
            'j': (0, 0, 0, -1),
            'l': (0, 0, 0, 1),
            'u': (1, 0, 0, -1),
            ',': (-1, 0, 0, 0),
            '.': (-1, 0, 0, -1),
            'm': (-1, 0, 0, 1),
            'O': (1, -1, 0, 0),
            'I': (1, 0, 0, 0),
            'J': (0, 1, 0, 0),
            'L': (0, -1, 0, 0),
            'U': (1, 1, 0, 0),
            '<': (-1, 0, 0, 0),
            '>': (-1, -1, 0, 0),
            'M': (-1, 1, 0, 0),
            't': (0, 0, -1, 0),
            'b': (0, 0, 1, 0),
        }

        self.speedBindings = {
            'q': (1.1, 1.1),
            'z': (.9, .9),
            'w': (1.1, 1),
            'x': (.9, 1),
            'e': (1, 1.1),
            'c': (1, .9),
        }

    @staticmethod
    def msg() -> str:        
        return """
        This node takes keypresses from the keyboard and publishes them
        as Twist/TwistStamped messages. It works best with a US keyboard layout.
        ---------------------------
        Moving around:
        u    i    o
        j    k    l
        m    ,    .

        For Holonomic mode (strafing), hold down the shift key:
        ---------------------------
        U    I    O
        J    K    L
        M    <    >

        t : up (+z)
        b : down (-z)
        1: takeoff
        2: land

        anything else : stop

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%

        CTRL-C to quit
        """
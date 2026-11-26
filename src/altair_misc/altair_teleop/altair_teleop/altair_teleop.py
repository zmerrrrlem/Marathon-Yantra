from typing import Any, Optional, Union

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.duration import Duration
from sensor_msgs.msg import Joy

import copy

# Joy axes ======
# stick_x = 0
# stick_y = 1
# stick_left = 2
# stick_right: 5

# Button =======
# kotak: 3
# segitiga: 2
# lingkaran: 1
# silang: 0
# R1: 5
# R2: 7
# L1: 4
# L2: 6
# start: 9
# menu: 8
# up: 10
# right: 13
# down: 11
# left: 12  

class AltairTeleop(Node):

    def __init__(self):
        """Constructor"""

        super().__init__("altair_teleop")
        
        self.declare_parameter("type", "custom")
        controller_type = ["custom"]

        for controller in controller_type:
            self.declare_parameter(f"{controller}.walking.gain_x", 0.0)
            self.declare_parameter(f"{controller}.walking.stick_x", 0)
            self.declare_parameter(f"{controller}.walking.gain_y", 0.0)
            self.declare_parameter(f"{controller}.walking.stick_y", 0)
            self.declare_parameter(f"{controller}.walking.stick_left", 0)
            self.declare_parameter(f"{controller}.walking.stick_right", 0)
            self.declare_parameter(f"{controller}.walking.gain_turn", 0.0)
            self.declare_parameter(f"{controller}.walking.btn_full_stop", 0)
            self.declare_parameter(f"{controller}.kick.btn_walk_kick", 0)

        selected_controller_type = self.get_parameter("type").get_parameter_value().string_value
        self.config = {}
        self.config["walking"] = {
            "gain_x": self.get_parameter(f"{selected_controller_type}.walking.gain_x")
            .get_parameter_value()
            .double_value,
            "stick_x": self.get_parameter(f"{selected_controller_type}.walking.stick_x")
            .get_parameter_value()
            .integer_value,
            "gain_y": self.get_parameter(f"{selected_controller_type}.walking.gain_y")
            .get_parameter_value()
            .double_value,
            "stick_y": self.get_parameter(f"{selected_controller_type}.walking.stick_y")
            .get_parameter_value()
            .integer_value,
            "stick_left": self.get_parameter(f"{selected_controller_type}.walking.stick_left")
            .get_parameter_value()
            .integer_value,
            "gain_turn": self.get_parameter(f"{selected_controller_type}.walking.gain_turn")
            .get_parameter_value()
            .double_value,
            "btn_full_stop": self.get_parameter(f"{selected_controller_type}.walking.btn_full_stop")
            .get_parameter_value()
            .integer_value,
        }

        self.config["kick"] = {
            "btn_walk_kick": self.get_parameter(f"{selected_controller_type}.kick.btn_walk_kick")
            .get_parameter_value()
            .integer_value,
        }

        self.walk_msg = Twist()
        self.last_walk_msg = Twist()
        self.walk_msg.linear.x = 0.0
        self.walk_msg.linear.y = 0.0
        self.walk_msg.linear.z = 0.0

        self.walk_msg.angular.x = 0.0
        self.walk_msg.angular.y = 0.0
        self.walk_msg.angular.z = 0.0

        # Subscriber
        self.create_subscription(Joy, "joy", self.joy_cb, 1)
        # Publisher 
        self.walk_publisher = self.create_publisher(Twist, "cmd_vel", 1)
        self.kick_publisher = self.create_publisher(Bool, "kick", 1)

        

    def denormalize_joy(self, gain, axis, msg: Joy, deadzone=0.0):
        if abs(msg.axes[axis]) > deadzone:
            return gain * msg.axes[axis]
        else:
            return 0
        
    def joy_cb(self, msg: Joy):
        # forward and sideward walking with left joystick
        self.walk_msg.linear.x = float(
            self.denormalize_joy(self.config["walking"]["gain_x"], self.config["walking"]["stick_x"], msg, 0.01)
        )
        print(self.walk_msg.linear.x)
        self.walk_msg.linear.y = float(
            self.denormalize_joy(self.config["walking"]["gain_y"], self.config["walking"]["stick_y"], msg, 0.01)
        )

        # angular walking with shoulder buttons
        turn = msg.axes[self.config["walking"]["stick_left"]]

        turn *= self.config["walking"]["gain_turn"]

        if turn != 0:
            self.walk_msg.angular.z = float(turn)
        else:
            self.walk_msg.angular.z = 0.0

        # Perform full stop (finish step but do not move afterwards)
        if msg.buttons[self.config["walking"]["btn_full_stop"]] == 1:
            self.walk_msg.linear.x = 0.0
            self.walk_msg.linear.y = 0.0
            self.walk_msg.linear.z = 0.0
            self.walk_msg.angular.z = 0.0
            self.walk_msg.angular.x = -1.0
            # Publish seperatly as we only want to publish the pressing not the releasing
            self.walk_publisher.publish(self.walk_msg)
        else:
            # Undo full stop
            self.walk_msg.angular.x = 0.0

        # only publish changes
        if self.walk_msg.linear != self.last_walk_msg.linear or self.walk_msg.angular.z != self.last_walk_msg.angular.z:
            self.walk_publisher.publish(self.walk_msg)
        self.last_walk_msg = copy.deepcopy(self.walk_msg)


        if msg.buttons[self.config["kick"]["btn_walk_kick"]]:
            self.kick_publisher.publish(Bool(data=True))


def main():

    rclpy.init()
    node = AltairTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

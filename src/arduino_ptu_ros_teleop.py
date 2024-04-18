#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickPanTiltController:
    def __init__(self):
        rospy.init_node('joystick_pantilt_controller')
        self.cmd_pub = rospy.Publisher('/ptu/cmd_vel', Twist, queue_size=10)
        self.joy_sub = rospy.Subscriber('/joy_teleop/joy', Joy, self.joy_callback)
        self.deadman_button = 4  # Adjust this if the deadman button is different

    def joy_callback(self, msg):
        if msg.buttons[self.deadman_button]:
            twist_cmd = Twist()
            twist_cmd.angular.z = msg.axes[3]  # Pan control
            twist_cmd.angular.y = msg.axes[4]  # Tilt control
            self.cmd_pub.publish(twist_cmd)

if __name__ == '__main__':
    try:
        JoystickPanTiltController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

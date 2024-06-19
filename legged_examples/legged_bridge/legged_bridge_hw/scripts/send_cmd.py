#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading

class KeyboardController:
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist_msg = Twist()
        self.rate = rospy.Rate(10)

    def on_press(self, key):
        try:
            if key.char == 'q':
                rospy.signal_shutdown("Quit")
            else:
                if key.char == 'w':
                    self.twist_msg.linear.x = 0.35
                elif key.char == 's':
                    self.twist_msg.linear.x = -0.35
                else:
                    self.twist_msg.linear.x = 0.0

                if key.char == 'a':
                    self.twist_msg.angular.z = 0.35
                elif key.char == 'd':
                    self.twist_msg.angular.z = -0.35
                else:
                    self.twist_msg.angular.z = 0.0
        except AttributeError:
            pass

    def on_release(self, key):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0

def ros_publish():
    while not rospy.is_shutdown():
        controller.publisher.publish(controller.twist_msg)
        controller.rate.sleep()


if __name__ == '__main__':
    rospy.init_node("keyboard_control")
    controller = KeyboardController()

    thread = threading.Thread(target=ros_publish)
    thread.start()

    listener = keyboard.Listener(on_press=controller.on_press, on_release=controller.on_release)
    listener.start()
    while not rospy.is_shutdown():
        pass

    listener.stop()
    listener.join()
    
    thread.join()

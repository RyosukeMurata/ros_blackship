#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

REMAPPABLE_NODE_NAME = 'js_commander1'
PARAM_NAME_TWISTSTAMPED_TOPIC = '/blackship_input'
PARAM_NAME_JOYSTICK_TOPIC = '/joystick_topic'


class JoyStickTwistStampedPublisher(object):
    def __init__(self):
        self._joy_data = Joy()
        self._twiststamped = TwistStamped()
        self._publisher = rospy.Publisher(rospy.get_param(PARAM_NAME_TWISTSTAMPED_TOPIC),
                                          TwistStamped, queue_size=1)
        self.is_activated = False

    def activate(self):
        rospy.Subscriber(rospy.get_param(PARAM_NAME_JOYSTICK_TOPIC, default='joy_node'),
                         Joy, self._joy_callback)
        self.is_activated = True

    def _joy_callback(self, joy_data):
        self._joy_data = joy_data

    def publish_twiststamped(self):
        self._twiststamped.twist.linear.x = self._joy_data.axes[1]
        self._twiststamped.twist.angular.z = self._joy_data.axes[0]
        self._publisher.publish(self._twiststamped)


# -------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node(REMAPPABLE_NODE_NAME, anonymous=True)
    rate = rospy.Rate(100)  # hz

    twiststamped_pub = JoyStickTwistStampedPublisher()
    twiststamped_pub.activate()

    while not rospy.is_shutdown():
        twiststamped_pub.publish_twiststamped()
        rate.sleep()
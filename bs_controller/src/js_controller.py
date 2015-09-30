#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

REMAPPABLE_NODE_NAME = 'js_commander'
PARAM_NAME_TWISTSTAMPED_TOPIC = '/blackship_input'
PARAM_NAME_JOYSTICK_TOPIC = '/joystick_topic'


class JoyStickTwistStampedPublisher(object):
    def __init__(self):
        self._joy_data = Joy()
        self._twist_stamped = TwistStamped()
        self._publisher = rospy.Publisher('bs_input', TwistStamped, queue_size=1)
        self.is_activated = False
        self._scale_v = rospy.get_param('~scale_v')
        self._scale_w = rospy.get_param('~scale_w')

    def activate(self):
        rospy.Subscriber('joy', Joy, self._joy_callback)
        self.is_activated = True

    def _joy_callback(self, joy_data):
        self._update_scales([joy_data.buttons[0], joy_data.buttons[2],
                             joy_data.buttons[3], joy_data.buttons[1]])
        self._update_inputs(joy_data.axes[1], joy_data.axes[0])

    def _update_scales(self, btns):
        v_up = 0.01 if btns[0] == 1 else -0.01 if btns[1] == 1 else 0.0
        w_up = 0.01 if btns[2] == 1 else -0.01 if btns[3] == 1 else 0.0
        self._scale_v += v_up
        self._scale_w += w_up

    def _update_inputs(self, up_down_axis, left_right_axis):
        self._twist_stamped.twist.linear.x = self._scale_v * up_down_axis
        self._twist_stamped.twist.angular.z = self._scale_w * left_right_axis

    def publish_twiststamped(self):
        self._publisher.publish(self._twist_stamped)


# -------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node(REMAPPABLE_NODE_NAME, anonymous=True)
    rate = rospy.Rate(100)  # hz

    twiststamped_pub = JoyStickTwistStampedPublisher()
    twiststamped_pub.activate()

    while not rospy.is_shutdown():
        twiststamped_pub.publish_twiststamped()
        rate.sleep()
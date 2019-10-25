#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tello teleoperation with tello_driver.py

Controls only work during the override state, so:

    - LB: enable override
    - RB: disable override

General controls:

    - Rx: Pitch
    - Ry: Roll
    - Lx: Altitude
    - Ly: Yaw

    - Y: Takeoff
    - B: Land
    - RT: Emergency

    - D-pad up: increase speed
    - D-pad down: decrease speed

"""

import rospy
from std_msgs.msg import Empty, String, Bool, Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Joy   # http://wiki.ros.org/joy
from tello_driver.srv import Takeoff, TakeoffResponse
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import time
import sys

def get_frame(msg):
    global frame
    global cv_bridge
    global drone_state
    global override

    try:
        frame = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        if frame is not None:
            hud = frame.copy()

            if drone_state:
                text = "ALT: {}   BAT: {}   TMP LO: {}   TMP HI: {}".format(
                        drone_state['h'], drone_state['bat'], drone_state['templ'], drone_state['temph'])
            else:
                text = "NO SENSOR DATA"

            if override:
                state = "MANUAL"
            else:
                state = "AUTO"

            cv2.putText(hud, text, (5, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
            cv2.putText(hud, state, (frame.shape[1] - 130, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (40,255,0), 2)
            cv2.imshow('FPV', hud); cv2.waitKey(1);

    except CvBridgeError as e:
        print(e)


def get_drone_state(msg):
    global drone_state
    drone_state = dict(item.split(":") for item in msg.data.split(";"))

def gamepad_handler(msg):
    global takeoff
    global land
    global kill
    global override_set
    global override_out
    global override
    global vel_twist
    global speed
    global takeoff_service
    global land_service
    global last_recv

    last_recv = msg.header.stamp.secs

    if msg.buttons[4]: # LB, override
        if not override:
            override_set.publish(Bool(data=True))
            print("[INFO] Setting control to MANUAL")
        override = True

    if msg.buttons[5]: # RB, automatic
        vel_twist = Twist()
        override_out.publish(vel_twist)
        if override:
            override_set.publish(Bool(data=False))
            print("[INFO] Setting control to AUTO")
        override = False
        return

    if override:

        # Handle buttons
        if msg.buttons[3]:   # Y
            takeoff_service()
        if msg.buttons[1]:   # B
            land_service()
        if msg.axes[7] == 1:  # D-pad up
            speed = np.min([speed+20,100])
            print("[INFO] Setting speed to {}".format(speed))
        if msg.axes[7] == -1: # D-pad down
            speed = np.max([speed-20,20])
            print("[INFO] Setting speed to {}".format(speed))

        # Handle axes
        vel_twist.linear.x = speed*msg.axes[4]
        vel_twist.linear.y = -speed*msg.axes[3]
        vel_twist.linear.z = speed*msg.axes[1]
        vel_twist.angular.z = -speed*msg.axes[0]

        override_out.publish(vel_twist)

    if msg.axes[5] < -0.5: # RT, kill motors
        kill.publish(Empty())
        cv2.destroyAllWindows()
        sys.exit(0)


if __name__ == '__main__':

    frame = None
    drone_state = ""
    speed = 100
    override = False
    last_recv = time.time()
    cv_bridge = CvBridge()
    vel_twist = Twist()

    kill = rospy.Publisher('kill', Empty, queue_size=1)
    override_set = rospy.Publisher('man_override', Bool, queue_size=1, latch=True)
    override_out = rospy.Publisher('overrride_rc_in', Twist, queue_size=1)

    rospy.Subscriber('camera/image_raw', Image, get_frame)
    rospy.Subscriber('drone_state', String, get_drone_state)
    rospy.Subscriber('joy', Joy, gamepad_handler)

    takeoff_service = rospy.ServiceProxy('takeoff', Takeoff)
    land_service = rospy.ServiceProxy('land', Takeoff)

    rospy.init_node('tello_teleop')
    rate = rospy.Rate(1)

    # Watch for idleness
    while not rospy.is_shutdown():

        if override and time.time()-last_recv > 3.0:
            vel_twist = Twist()
            override_out.publish(vel_twist)

        rate.sleep()

    land_service()
    cv2.destroyAllWindows()

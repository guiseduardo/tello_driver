#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tello teleoperation with tello_driver.py

Controls:
    - ...

"""

import rospy
from std_msgs.msg import Empty, String, Bool, Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Joy   # http://wiki.ros.org/joy
from mission_control.srv import Takeoff, TakeoffResponse
from vision_utils.msg import Qrcodes
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2
import time
import sys

def get_frame(msg):
    global frame
    global cv_bridge
    global drone_state
    global qr_pos
    global qr_text
    global tag_draw
    global window_draw
    global shelf_draw
    global tag_text
    global drawings
    global STATE

    try:
        frame = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        if frame is not None:
            hud = frame.copy()

            if drawings:
                if tag_draw is not None and STATE=="AUTO":
                    try:
                        x,y,w,h = tag_draw
                        cv2.rectangle(hud, (x,y), (x+w,y+h), (0,255,0), 3)
                        cv2.circle(hud, (int(x+w/2),int(y+h/2)), 5, (0,0,255), 5)
                        cv2.putText(hud, tag_text, (x, y - 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,40,0), 2)
                        cv2.putText(hud, "possible tag", (x, y+h + 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                    except:
                        pass
                    ref = (int(960/2),int(2*720/3)+15)
                    cv2.circle(hud, ref, 80, (100,200,0), 5)

                save_pos = qr_pos
                if save_pos is not None and save_pos.shape[0] and STATE=="AUTO":
                    for qr in range(save_pos.shape[0]):
                        x,y,w,h = save_pos[qr]
                        try:
                            cv2.rectangle(hud, (x,y), (x+w, y+h), (0,50,255), 2)
                            cv2.putText(hud, qr_text[qr], (x, y - 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,120,255), 2)
                        except:
                            pass

                if window_draw is not None and STATE=="AUTO_WINDOW":
                    try:
                        x,y,w,h = window_draw
                        cv2.rectangle(hud, (x,y), (x+w,y+h), (0,200,180), 3)
                        cv2.circle(hud, (int(x+w/2),int(y+h/2)), 5, (255,60,0), 5)
                        cv2.putText(hud, "possible window", (x, y+h + 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,60,0), 1)
                    except:
                        pass

                if shelf_draw is not None and STATE=="AUTO_SHELF":
                    try:
                        x,y,w,h = shelf_draw
                        cv2.rectangle(hud, (x,y), (x+w,y+h), (200,120,0), 3)
                        cv2.circle(hud, (int(x+w/2),int(y+h/2)), 5, (0,120,255), 5)
                        cv2.putText(hud, tag_text, (x, y - 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,120,255), 2)
                        cv2.putText(hud, "possible shelf code", (x, y+h + 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,120,255), 1)
                    except:
                        pass

            if drone_state:
                text = "ALT: {}   BAT: {}   TMP LO: {}   TMP HI: {}".format(
                        drone_state['h'], drone_state['bat'], drone_state['templ'], drone_state['temph'])
            else:
                text = "NO SENSOR DATA"
            cv2.putText(hud, text, (5, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)

            cv2.imshow('FPV', hud); cv2.waitKey(1);
    except CvBridgeError as e:
        print(e)


def get_drone_state(msg):
    global drone_state
    drone_state = dict(item.split(":") for item in msg.data.split(";"))

def tag_cb(msg):
    global tag_draw
    tag_draw = msg.data

def tag_read(msg):
    global tag_text
    tag_text = msg.data

def qr_cb(msg):
    global qr_pos
    global qr_text
    if msg.messages:
        qr_pos = np.reshape(msg.brectangle,(-1,4))
        qr_text = msg.messages.split(';')[:-1]

def window_cb(msg):
    global window_draw
    window_draw = msg.data

def shelf_cb(msg):
    global shelf_draw
    shelf_draw = msg.data

def state_cb(msg):
    global STATE
    STATE = msg.data

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

    if msg.buttons[4]:
        if not override:
            override_set.publish(Bool(data=True))
        override = True

    if msg.buttons[5]:
        vel_twist = Twist()
        override_out.publish(vel_twist)
        if override:
            override_set.publish(Bool(data=False))
        override = False
        return

    if override: # LT, override

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

    # FPV type
    drawings = 1
    STATE = "AUTO"

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
    rospy.Subscriber('current_state', String, state_cb)
    rospy.Subscriber('joy', Joy, gamepad_handler)

    tag_draw = None
    tag_text = ""
    window_draw = None
    shelf_draw = None
    qr_pos = None
    qr_text = []

    rospy.Subscriber('green/brectangle', Int32MultiArray, tag_cb)
    rospy.Subscriber('yellow/brectangle', Int32MultiArray, window_cb)
    rospy.Subscriber('blue/brectangle', Int32MultiArray, shelf_cb)
    rospy.Subscriber('qrcodes/data', Qrcodes, qr_cb)
    rospy.Subscriber('tag_text', String, tag_read)

    takeoff_service = rospy.ServiceProxy('takeoff', Takeoff)
    land_service = rospy.ServiceProxy('land', Takeoff)

    rospy.init_node('tello_teleop')
    rate = rospy.Rate(1)

    # Watch for idleness
    while not rospy.is_shutdown():

        if override and time.time()-last_recv > 0.4:
            vel_twist = Twist()
            override_out.publish(vel_twist)

        rate.sleep()

    land_service()
    cv2.destroyAllWindows()

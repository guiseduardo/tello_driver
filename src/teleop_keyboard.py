#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tello teleoperation with tello_driver.py

Controls:
    - W A S D:     move 2D
    - Q E:         yaw
    - Tab Lctrl    up/down
    - Z            takeoff
    - X            land
    - Esc          kill motors
    - A            switch auto/manual

"""

import rospy
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tello_driver.srv import Takeoff, TakeoffResponse

import numpy as np
import cv2
import time
import sys
import pygame
import pygame.display
import pygame.key
import pygame.locals
import os

def get_frame(msg):
    global frame
    global cv_bridge
    global drone_state
    global override
    global hud
    global screen

    try:
        frame = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        screen.fill([0,0,0])

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
            #cv2.imshow('FPV', hud); cv2.waitKey(1);

            # Write to pygame screen
            hud = cv2.cvtColor(hud, cv2.COLOR_BGR2RGB)
            hud = np.rot90(hud)
            hud = cv2.flip(hud, 0)
            hud = pygame.surfarray.make_surface(hud)
            screen.blit(hud, (0,0))
            pygame.display.update()

    except CvBridgeError as e:
        print(e)


def get_drone_state(msg):
    global drone_state
    drone_state = dict(item.split(":") for item in msg.data.split(";"))


if __name__ == '__main__':

    frame = None
    hud = None
    drone_state = ""
    speed = 100
    cv_bridge = CvBridge()
    last_recv = time.time()
    override = False
    vel_twist = Twist()

    pygame.init()
    pygame.display.init()
    pygame.display.set_caption("Tello FPV")
    screen = pygame.display.set_mode((960, 720))

    rospy.init_node('tello_teleop')

    rospy.Subscriber('camera/image_raw', Image, get_frame)
    rospy.Subscriber('drone_state', String, get_drone_state)

    kill = rospy.Publisher('kill', Empty, queue_size=1)
    override_set = rospy.Publisher('man_override', Bool, queue_size=1, latch=True)
    vel_pub = rospy.Publisher('overrride_rc_in', Twist, queue_size=1)
    takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
    takeoff_service = rospy.ServiceProxy('takeoff', Takeoff)
    land_service = rospy.ServiceProxy('land', Takeoff)

    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            rate.sleep()

            for ev in pygame.event.get():
                if ev.type == pygame.locals.KEYDOWN:
                    k = pygame.key.name(ev.key)

                    # Operations
                    if k == 'escape':
                        kill.publish(Empty())
                        sys.exit(0)
                    elif k == 'z':
                        takeoff_service()
                    elif k == 'x':
                        land_service()
                    elif k == 'a':
                        override = not override
                        override_set.publish(Bool(data=override))
                        state = "MANUAL" if override else "AUTO"
                        print("[INFO] Setting control to "+state)

                    # Twists
                    elif k == 'w':
                        vel_twist.linear.x = speed
                    elif k == 's':
                        vel_twist.linear.x = -speed
                    elif k == 'a':
                        vel_twist.linear.y = -speed
                    elif k == 'd':
                        vel_twist.linear.y = speed
                    elif k == 'q':
                        vel_twist.angular.z = -speed
                    elif k == 'e':
                        vel_twist.angular.z = speed
                    elif k == 'tab':
                        vel_twist.linear.z = speed
                    elif k == 'left ctrl':
                        vel_twist.linear.z = -speed

                    # Set speed
                    elif k == 'up':
                        speed = np.min([speed+20,100])
                        print("[INFO] Setting speed to {}".format(speed))
                    elif k == 'down':
                        speed = np.max([speed-20,0])
                        print("[INFO] Setting speed to {}".format(speed))

                    if override:
                        vel_pub.publish(vel_twist)


                elif ev.type == pygame.locals.KEYUP:
                    k = pygame.key.name(ev.key)
                    if k in 'wasdqe' or k=='tab' or k=='left ctrl':
                        vel_twist = Twist()
                        vel_pub.publish(vel_twist)

            # 'while' level
            #vel_pub.publish(vel_twist)

#            if frame is not None:
#                cv2.imshow('FPV',frame)


    except Exception as e:
        print(str(e))

    finally:
        land_service()
        cv2.destroyAllWindows()
        sys.exit(1)
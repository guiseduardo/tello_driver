#!/usr/bin/env python3
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
    
"""

import rospy
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

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
    try:
        frame = cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
    except CvBridgeError as e:
        print(e)
    
    
def get_drone_state(msg):
    global drone_state
#    drone_state = dict(item.split(":") for item in msg.split(";"))
    pass

if __name__ == '__main__':
    
    frame = None
    drone_state = ""
    speed = 100
    cv_bridge = CvBridge()
    vel_twist = Twist()
    pos_twist = Twist()
    
    rospy.Subscriber('camera/image_raw', Image, get_frame)
    rospy.Subscriber('drone_state', String, get_drone_state)
    
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pos_pub = rospy.Publisher('cmd_pos', Twist, queue_size=1)
    takeoff = rospy.Publisher('takeoff', Empty, queue_size=1)
    land = rospy.Publisher('land', Empty, queue_size=1)
    kill = rospy.Publisher('kill', Empty, queue_size=1)
        
    rospy.init_node('tello_teleop')
    rate = rospy.Rate(5)
    
    pygame.init()
    pygame.display.init()
    pygame.display.set_mode((300, 300))
    
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            
            for ev in pygame.event.get():
                if ev.type == pygame.locals.KEYDOWN:
                    k = pygame.key.name(ev.key)
                    
                    # Operations
                    if k == 'escape':
                        kill.publish(Empty())
                        exit(0)
                    elif k == 'z':
                        takeoff.publish(Empty())
                    elif k == 'x':
                        land.publish(Empty())
                        
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
                        vel_twist.angular.z = -2*speed
                    elif k == 'e':
                        vel_twist.angular.z = 2*speed
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
        land.publish(Empty())
        cv2.destroyAllWindows()
        exit(1)
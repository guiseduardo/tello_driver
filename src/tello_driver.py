#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tello driver node

Read the docs: (TODO) tello_driver_doc.md
"""

import rospy
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from mission_control.srv import Takeoff, TakeoffResponse
from cv_bridge import CvBridge

import numpy as np
import socket
import cv2
import time
import sys
from threading import Thread


class Tello(object):

    def __init__(self):
        # Initialize a shutdown signal for every thread
        self._shutdown = False

        ### Create a UDP socket
        self.tello_address = ('192.168.10.1', 8889)

        # Command receiver thread initializer
        self.commsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.commsock.bind(("0.0.0.0",8889))
        self.commsock.settimeout(10.0)
        self.commRecvThread = Thread(target=self.__commrecv)
        self.commRecvThread.daemon = True
        self.commRecvThread.start()

        # State receiver thread initializer
        self.stsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stsock.bind(("0.0.0.0",8890))
        self.stsock.settimeout(10.0)
        self.stRecvThread = Thread(target=self.__strecv)
        self.stRecvThread.daemon = True
        self.stRecvThread.start()

        # Initialize variables
        self.commrecv = None
        self.strecv = None
        self.imrecv = None

        # Command queue
        self.queue_count_w = 0
        self.queue_count_r = 0
        self.queue_maxsize = 6
        self.cqueue = [None]*self.queue_maxsize
        self.queueThread = Thread(target=self.__queue_comm)
        self.queueThread.start()

        # Push initialization commands to queue
        self.cqueue[0] = "command"
        self.cqueue[1] = "streamon"
        self.queue_count_w = 2
        self.okack = False # Wait for 'ok' ACK
        self.toffack = False # Wait for 'ok' ACK (from takeoff)

        # Read commands are stored in this dictionary,
        # according to the command string passed, like this:
        self.recvd = {'none?': 0}

        # Initialize ROS subs, pubs and node
        self.cv_bridge = CvBridge() # ROS opencv bridge
        self.vel_twist = Twist()
        self.pos_twist = Twist()
        self.override_flag = False
        self.override_ctrl = Twist()

        self.stream_pub = rospy.Publisher('camera/image_raw', Image, queue_size=1)
        self.state_pub = rospy.Publisher('drone_state', String, queue_size=1)

        rospy.Subscriber('cmd_vel', Twist, self.__cmd_vel_callback)
        rospy.Subscriber('cmd_pos', Twist, self.__cmd_pos_callback)
        rospy.Subscriber('kill', Empty, self.kill)
        rospy.Subscriber('man_override', Bool, self.__set_man_override)
        rospy.Subscriber('overrride_rc_in', Twist, self.__man_override_ctrl_callback)

        self.takeoff_service = rospy.Service('takeoff', Takeoff, self.takeoff)
        self.land_service = rospy.Service('land', Takeoff, self.land)

        rospy.init_node('Tello')

        # Start video thread
        self.stream = None
        self.capst = None
        self.open_video_stream()

        # Thread to parse sensors
        self.sparserThread = Thread(target=self.__sparser)
        self.sparserThread.start()
        self.keepAlive = Thread(target=self.__keepalive)
        self.keepAlive.start()

        # Spin until que class quits
        rospy.spin()
        self.quit()


    ##### Receiver threads

    # Command receiver thread
    def __commrecv(self):
        while not rospy.is_shutdown():
            try:
                self.commrecv,_ = self.commsock.recvfrom(1024)
                #print(data.decode(encoding="utf-8"))
            except Exception as e:
                print('[INFO] Comm socket closed. Quitting . . .')
                print('[DEBUG] Socket exception:')
                print(e)
                break

    # State receiver thread
    def __strecv(self):
        while not rospy.is_shutdown():
            try:
                self.strecv,_ = self.stsock.recvfrom(2048)
                #print(data.decode(encoding="utf-8"))
            except:
                print('[INFO] State socket closed. Quitting . . .')
                break

    ##### Video receiver thread + aux functions
    def __videorecv(self):
        time.sleep(2) # Wait initialization
        while not self.okack:
            time.sleep(1)
        print("[INFO] Opening UDP video stream")
        self.stream = cv2.VideoCapture("udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=500000",cv2.CAP_FFMPEG)
#        dropframes = 300
        while not rospy.is_shutdown():
            self.capst, self.frame = self.stream.read()

            # Publish image
            if self.capst is not None:
                self.stream_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.frame, 'bgr8'))
#                if dropframes < 0:
#                    self.stream_pub.publish(self.cv_bridge.cv2_to_imgmsg(self.frame, 'bgr8'))
#                    time.sleep(1.0/30)
#                else:
#                    dropframes -= 1

            if self._shutdown or rospy.is_shutdown():
                print("[INFO] Closing video stream")
                self.stream.release()
                break

    def open_video_stream(self):
        self.videoThread = Thread(target=self.__videorecv)
        self.videoThread.start()

    ##### Command handlers
    def __push_queue(self, comm):
        # Push command to next position in queue
        if self.cqueue[self.queue_count_w] is None:
            self.cqueue[self.queue_count_w] = comm
            self.queue_count_w += 1
            if self.queue_count_w == self.queue_maxsize: self.queue_count_w = 0
#        else:
#            print("[WARNING] Command queue full, dropping packet")

    def __queue_comm(self):
        # Queue is checked every 15ms. If queue is not empty,
        # pull next command
        while not rospy.is_shutdown():

            if self._shutdown or rospy.is_shutdown(): break

            comm = self.cqueue[self.queue_count_r]
            if comm is not None:
                self.commsock.sendto(comm.encode("utf-8"),("192.168.10.1",8889))

                # If response is awaited: hold and store on self.recvd
                if "?" in comm:
                    while self.commrecv is None:
                        time.sleep(.005)
                    try:
                        ack = str(self.commrecv.decode("utf-8"))
                    except:
                        print("[WARNING] Error acknowledging response for {}".format(comm))

                    if ack.isdigit(): ack = int(ack)
                    else: ack = ack[:-4]

                    self.recvd[comm] = ack
                    self.commrecv = None

                # Wait but don't store for these cases, set flag self.okack
                if comm == "command" or comm == "streamon":
                    while self.commrecv is None:
                        time.sleep(.005)
                    print("Initializing <{}> . . . OK!".format(comm))
                    self.commrecv = None
                    self.okack = True

                # Hold queue until takeoff or landing is complete
                if comm == "takeoff" or comm == "land":
                    while self.commrecv is None:
                        time.sleep(.005)
                    self.commrecv = None
                    self.toffack = True

                self.cqueue[self.queue_count_r] = None
                self.queue_count_r += 1
                if self.queue_count_r == self.queue_maxsize: self.queue_count_r = 0
            time.sleep(0.015)

    ##### Sensor parser: every loop send to queue message requests.
    # With ROS, this thread publishes the values from the sensor dict.
    def __sparser(self):
        while not self.okack:
            time.sleep(.1)
        while not rospy.is_shutdown():
            if self._shutdown or rospy.is_shutdown(): break
            if self.strecv is not None:
                self.strstrecv = self.strecv.decode("utf-8")[:-4]
                self.state_pub.publish(self.strstrecv)
#                print(dict(item.split(":") for item in self.strstrecv.split(";")))
            time.sleep(.95)

    # Substitute __sparser in keeping the drone alive pinging every second
    def __keepalive(self):
        while not self.okack:
            time.sleep(1)
        while not rospy.is_shutdown():
            if self._shutdown: break
            self.__push_queue("tof?")
            time.sleep(1)


    ##### Common callbacks
    def takeoff(self, req):
        self.__push_queue("takeoff")
        timeout = 8
        while not self.toffack:
            time.sleep(.5)
            timeout -= 1
            if not timeout: return TakeoffResponse(ack=False)
        self.toffack = False
        return TakeoffResponse(ack=True)


    def land(self, req):
        self.__push_queue("land")
        # I know, these are the takeoff variables, but
        # they serve the same purpose
        timeout = 8
        while not self.toffack:
            time.sleep(.5)
            timeout -= 1
            if not timeout: self.kill(Empty())
        self.toffack = False
        return TakeoffResponse(ack=True)

    def kill(self, msg):
        # Override queue and kill motors
        self.commsock.sendto("emergency".encode("utf-8"),("192.168.10.1",8889))
        time.sleep(.02)
        self.quit()

    ##### Twist callbacks
    def __cmd_pos_callback(self, msg):
        # Twist x, y, and z are S/E
        # Twist pitch/roll are ignored

        # Check x
        cm = 0.8929*msg.linear.x   # Compensate drift
        if cm > 0:
            cm = int(np.max([np.min([cm,500]),20])) # Bound input
            self.__push_queue("forward "+str(cm))
        elif cm < 0:
            cm = int(np.max([np.min([abs(cm),500]),20]))
            self.__push_queue("back "+str(cm))

        # Check y
        cm = 0.8427*msg.linear.y   # Compensate drift
        if cm > 0:
            cm = int(np.max([np.min([cm,500]),20]))
            self.__push_queue("right "+str(cm))
        elif cm < 0:
            cm = int(np.max([np.min([abs(cm),500]),20]))
            self.__push_queue("left "+str(cm))

        # Check z
        cm = msg.linear.z   # We will compensate drift sign(cm)-wise
        if cm > 0:
            cm = int(np.max([np.min([1.25*cm,500]),20]))
            self.__push_queue("up "+str(cm))
        elif cm < 0:
            cm = int(np.max([np.min([abs(1.4084*cm),500]),20]))
            self.__push_queue("down "+str(cm))

        # Check yaw
        cm = msg.angular.z
        if cm > 0:
            cm = int(np.max([np.min([cm,360]),1]))
            self.__push_queue("ccw "+str(cm))
        elif cm < 0:
            cm = int(np.max([np.min([abs(cm),360]),1]))
            self.__push_queue("cw "+str(cm))


    def __cmd_vel_callback(self, msg):

        # Bound inputs
        x = int(np.max([np.min([msg.linear.x,100]),-100]))
        y = int(np.max([np.min([msg.linear.y,100]),-100]))
        z = int(np.max([np.min([msg.linear.z,100]),-100]))
        w = int(np.max([np.min([msg.angular.z,100]),-100]))

        if not self.override_flag: self.__push_queue("rc %s %s %s %s"%(y,x,z,w))

    ##### Handle manual override
    def __set_man_override(self, msg):
        self.override_flag = msg.data

    def __man_override_ctrl_callback(self, msg):
        # Bound inputs
        x = int(np.max([np.min([msg.linear.x,100]),-100]))
        y = int(np.max([np.min([msg.linear.y,100]),-100]))
        z = int(np.max([np.min([msg.linear.z,100]),-100]))
        w = int(np.max([np.min([msg.angular.z,100]),-100]))

        if self.override_flag: self.__push_queue("rc %s %s %s %s"%(y,x,z,w))

    ##### Quit op and close threads
    def quit(self):
        print("[INFO] Landing . . .")
        # Override queue and land ASAP
        self.commsock.sendto("land".encode("utf-8"),("192.168.10.1",8889))
        self.commsock.close()
        self.stsock.close()
        print("[INFO] Quitting . . .")
        self._shutdown = True
        sys.exit(0)


if __name__ == '__main__':
    try:
        driver = Tello()
    except rospy.ROSInterruptException:
        driver.quit()
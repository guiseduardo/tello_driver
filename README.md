# ROS driver for the DJI Tello 

This is a work in progress implementation of a ROS node to communicate with a Tello Edu (SDK 2.0). It is still in early development, so it might crash and hang sometimes. Even so, the drone should land/stop if this node dies and/or the connection is shut off.

## To install

Just clone this to your catkin workspace and run `catkin_make`.

## Quickstart

Connect to the Tello access point, plug in a gamepad and run:

```sh
$ roslaunch tello_driver start_basics.launch
```

# Usage

## Takeoff and landing

Those are implemented as services. Ideally, you want to wait for an 'ack' after sending the command, but this is sometime missed.

Usage:

```python
from tello_driver.srv import Takeoff, TakeoffResponse
takeoff_relay = rospy.ServiceProxy('takeoff', Takeoff)
land_relay = rospy.ServiceProxy('land', Takeoff)
ack = takeoff_relay()   # or ack = land_relay()
```


## Emergency

Immediately cuts motors. Activated by sending an **Empty** message under the topic /kill. To do so:

```python
from std_msgs.msg import Empty
kill_switch = rospy.Publisher('kill', Empty, queue_size=1)
kill_switch.publish(Empty())
```


## RC commands

There are 2 channels for sending RC commands, they're both *Twist* topics:

- /cmd_vel
- /overrride_rc_in

Commands are sent to */cmd_vel* by default. To switch, set */man_override* to **True** (this is implemented in *teleop_gamepad.py*). Both channels bind commands to values between -100 and 100.


## Position commands

To send a fixed-length movement command, use the topic */cmd_pos*, also using *Twist*.

Commands for each direction should be sent separately and it is recomended to wait for action completion. The value to be sent is the desired distance to move in centimetres, bound between 20 and 500 cm.


## FPV

The front camera can be accessed via the topic */camera/image_raw*. This topic is a ROS bgr-8 image type. Use CVBridge to convert to a cv2 object:

```python
from cv_bridge import CvBridge
cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
```
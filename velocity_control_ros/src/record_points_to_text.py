#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('velocity_control_ros')
import rospy
import numpy as np
import time
import sys
import IPython
import datetime
import os

from mit_msgs.msg import MocapPosition
from geometry_msgs.msg import Vector3

filename = "eepos.txt"
try:
    os.remove(filename)
except:
    pass

def record(msg):
    x = msg.translational.x / 1000.0
    y = msg.translational.y / 1000.0
    z = msg.translational.z / 1000.0
    with open(filename, "a") as myfile:
        myfile.write(str(x) +' '+ str(y) +' ' + str(z) + "\n")
        
def recordEepose(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    with open(filename, "a") as myfile:
        myfile.write(str(x) +' '+ str(y) +' ' + str(z) + "\n")

rospy.init_node('record_points')
rospy.Subscriber("/end_effector_pose", Vector3, recordEepose)


rospy.spin()

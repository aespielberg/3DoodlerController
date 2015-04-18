#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import openravepy as orpy
import time

import IPython


def convert_to_real_youbot_joint_values(q):
    jointdiff = np.array([2.949606435870417,
                          1.1344640137963142,
                         -2.548180707911721,
                         1.7889624832941877,
                         2.923426497090502])
    return jointdiff + q

#np.set_printoptions(suppress=True)
#np.set_printoptions(precision=4)

env = orpy.Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('robots/kuka-youbot.robot.xml') 
#env.Load('/home/youbot/youbotmodel2/brsu-youBot-T/standalone/robots/youbot_5D_base_gripper.robot.xml') 
#env.Load('robots/kuka-youbot.zae') 
#env.SetDebugLevel(orpy.DebugLevel.Debug);
#ipshell()
robot = env.GetRobots()[0]
manip = robot.GetManipulators()[0]

IPython.embed()

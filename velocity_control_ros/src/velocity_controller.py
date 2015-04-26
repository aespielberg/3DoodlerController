#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('velocity_control_ros')
import rospy
import numpy as np
import time
import sys
import pickle
import random
import copy
import IPython
from itertools import izip,product,combinations,chain
import datetime

import openravepy as orpy
import youbotpy
from youbotpy import youbotik as yik
import GraspGenerator
import AnytimePlanner
import CollisionConstraints
import copy
import tfplugin
from brics_actuator.msg import JointVelocities, JointValue
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import sys



BACKUP_AMT = 0.05
LIFT_AMT = 0.3
MOVE_AMT = 0.1
MAX_VEL = 0.1
THRESH = 0.002
BIG_THRESH = 0.01
BAD_ITERS = 10
SPEED_FACTOR = 0.25 #empirical guesstimate at transforming joint to cartesian speed

kd = 0.
ki = 0.
kp = 1.
ki = 1.6
kd = 0.0002 #Been finding this really bad, maybe get rid of it





sim = False
use_vicon_for_start_poses = True

arm_names = []
for i in range(5):
    arm_names.append('arm_joint_' + str(i + 1))
    
unit = 's^-1 rad'

all_robot_names = ['drc1']
r = all_robot_names[0]
envfile = 'environments/floor.env.xml'

youbotenv = youbotpy.YoubotEnv(sim=sim,viewer=True,env_xml=envfile, \
                               youbot_names=all_robot_names, registered_objects=None)

env = youbotenv.env
youbots = youbotenv.youbots
if sim:
    for name in youbots:
        youbotenv.MoveGripper(name,0.01,0.01) # open grippers

yik.init(youbots[youbots.keys()[0]]) #   does not matter which robot
                                     # we use to initialize since 
                                     # all youbots have same kinematics.
grasp_generator = GraspGenerator.GraspGenerator(env)

offset = np.array([2.950, 1.1345, -2.5482, 1.7890, 2.9234])




robot_base_homes = {}
for r in all_robot_names:
    robot_base_homes[r] = youbots[r].GetTransform()
    
planners = {}
for r in all_robot_names:
    planners[r] = orpy.interfaces.BaseManipulation(youbots[r],plannername='BiRRT')



op_pose = np.eye(4)

rospy.init_node('velocity_controller')
pub = rospy.Publisher('/' + all_robot_names[0] + '/arm_1/arm_controller/velocity_command', JointVelocities, queue_size=1)
pos_pub = rospy.Publisher('/end_effector_pose', Vector3, queue_size=1)
extrude_pub_fast = rospy.Publisher('fast', Bool, queue_size=1)
extrude_pub_slow = rospy.Publisher('slow', Bool, queue_size=1)

def GetClosestArm(cur_arm, sol):
    best_dist = sys.maxint
    ret_i = 0
    for (i, v) in enumerate(sol):
        dist = np.linalg.norm(v - cur_arm, np.inf )
        if dist < best_dist:
            best_dist = dist
            ret_i = i
    #IPython.embed()
    return sol[ret_i]

def MoveArmTo(robot,goal,planner):
    try:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Verbose)
        with env:
            robot.SetActiveDOFs(range(5))
            traj = planner.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True, jitter=-0.01)
            robot.GetController().SetPath(traj)
        while not robot.GetController().IsDone():
            time.sleep(0.01)
    finally:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Info)
        
def getEndEffector():
    return youbots[r].GetManipulators()[0].GetEndEffectorTransform()
    
def createVelocity(vels):
    for i in range(len(vels)):
        vels[i] = float(vels[i])
    vels = np.array(vels)
    #clamp
    if np.linalg.norm(vels, 2) > MAX_VEL:
        vels /= np.linalg.norm(vels, 2)
        vels *=  MAX_VEL
    
    v = JointVelocities()
    for i in range(5):
        v.velocities.append(JointValue())
        v.velocities[i].joint_uri = arm_names[i]
        v.velocities[i].unit = unit
        v.velocities[i].value = float(vels[i])
    return v
    
def stop():
    v = createVelocity([0, 0, 0, 0, 0])
    pub.publish(v)

#start_config = np.array([2.9499957785497077, 1.334502240891718, -1.2181996753192461, 1.789004272867827, 2.9234068314087893]) - offset
start_config = np.array([2.9499957785497077, 1.44502240891718, -1.2181996753192461, 2.6, 2.9234068314087893]) - offset
print start_config
time.sleep(3.0)
robot = youbots[r]
MoveArmTo(robot,start_config,planners[r])
print 'init done'
time.sleep(2.0)


def startExtruding(fast)
    stopExtruding() #first, stop both fast and slow commands
    if fast: #then start the appropriate command
        extrude_pub_fast.pub(Bool(data=True))
    else:
        extrude_pub_slow.pub(Bool(data=False))
    
def stopExtruding()
    extrude_pub_fast.pub(Bool(data=False))
    extrude_pub_slow.pub(Bool(data=False))


def MoveStraight(velocity_factor, rel_diff):
    """
    Moves the end effector in a straight line.
    Velocity_factor - how fast to move the joints
    rel_diff - how far to move the end effector relative to its current location
    """
    velocity_factor *= SPEED_FACTOR
    
    transform = getEndEffector()
    #For debugging with rqt_plot:
    
    
    
    target_x = transform[0, 3] + rel_diff[0]
    target_y = transform[1, 3] + rel_diff[1]
    target_z = transform[2, 3] + rel_diff[2]
    target = copy.deepcopy(transform)
    target[0, 3] = target_x
    target[1, 3] = target_y
    target[2, 3] = target_z
    
    #initialize:
    diff = np.array([0., 0., 0., 0., 0.])
    integ = np.array([0., 0., 0., 0., 0.])
    itera = 0
    
    best_distance = sys.maxint
    
    
    
    
    while np.linalg.norm(getEndEffector()[:-1, 3] - target[:-1, 3], 2) > THRESH:
        
        timestamp = time.time()
        startExtruding(fast=True)
        
        
        cart_dist = np.linalg.norm(getEndEffector()[:-1, 3] - target[:-1, 3], 2)
        if cart_dist < best_distance:
            best_distance = cart_dist
        else:
            itera += 1
        
        
        """
        print 'end effector:'
        print getEndEffector()
        print 'target:'
        print target
        """
        print 'dist is: '
        print np.linalg.norm(getEndEffector()[:-1, 3] - target[:-1, 3], 2)
        
        
        current_arm = robot.GetDOFValues()[0:5]
        
        sub_target = copy.deepcopy(getEndEffector())
        pos_pub.publish(Vector3(x=sub_target[0, 3], y=sub_target[1, 3], z=sub_target[2, 3]))
        sub_target[0, 3] += (target[0, 3] - sub_target[0, 3])
        sub_target[1, 3] += (target[1, 3] - sub_target[1, 3])
        sub_target[2, 3] += (target[2, 3] - sub_target[2, 3])
 
        
        

        sol = yik.FindIKSolutions(robot, sub_target)
        closest_arm = GetClosestArm(current_arm, sol)
        old_error = diff
        
        
        
        
        #P-Term in PID controller
        
        diff = closest_arm - current_arm
        
        if itera > BAD_ITERS and cart_dist < BIG_THRESH: #if we're getting worse than the best for BAD_ITERS consecutive cycles and not a million miles away
            break
        
        
        dt = time.time() - timestamp
        print 'dt'
        print dt
        
        #I-Term in PID controller
        integ += diff * dt
        
        #D-Term in PID controller
        
        
        
        deriv = (diff - old_error) / dt

        vel = kp * diff + ki * integ + kd * deriv
        
        
        norm=np.linalg.norm(vel)
        if norm != 0:
            vel /= norm
        
        v = createVelocity(vel*velocity_factor)
        pub.publish(v)
        rospy.sleep(0.1)
        print 'speed is'
        print (np.linalg.norm(getEndEffector()[:-1, 3] - target[:-1, 3], 2) - cart_dist)/dt

        #MoveArmTo(robot, closest_arm, planners[r])

    stop()
    stopExtruding()
    print 'loop'
    
MoveStraight(0.1, np.array([-0.02, -0.02, 0]))


"""
def MoveEEStraight(velocity_factor,target,step):
    #vel_factor = 1.5
    while np.linalg.norm([target[2, 3] - getEndEffector()[2, 3]], 2) > THRESH:
        print np.linalg.norm([target[2, 3] - getEndEffector()[2, 3]], 2)
        current_arm = robot.GetDOFValues()[0:5]
        sub_target_z = getEndEffector()[2, 3] + step
        sub_target = copy.deepcopy(target)
        sub_target[2, 3] = sub_target_z
        sol = yik.FindIKSolutions(robot, sub_target)
        closest_arm = GetClosestArm(current_arm, sol)
        diff = closest_arm - current_arm
        v = createVelocity(velocity_factor*diff)
        v.velocities[0].value = 0.0
        pub.publish(v)
        rospy.sleep(0.1)
    stop()
    
def MoveClecoDownVel(velocity_factor,length=0.08):
    transform = getEndEffector()
    target = copy.deepcopy(transform)
    target[2, 3] -= length
    MoveEEStraight(velocity_factor,target,-0.005)
    
MoveClecoDownVel(6.0,0.04)
"""


while True:
    time.sleep(0.1)


#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import openravepy as orpy
import time

from Team import Team

from IPython.Shell import IPShellEmbed
ipshell = IPShellEmbed() 


def convert_to_real_youbot_joint_values(q):
    jointdiff = np.array([2.949606435870417,
                          1.1344640137963142,
                         -2.548180707911721,
                          1.7889624832941877,
                          2.923426497090502])
    return jointdiff + q



def GetAssemblyPose():
    # FIXME For now returns a fixed pose.
    pose = np.array([[1.0,  0.0,  0.0,  0.00],
                     [0.0,  0.0, -1.0,  0.00],
                     [0.0,  1.0,  0.0,  0.40],
                     [0.0,  0.0,  0.0,  1.0]])
    return pose

def FormTeam(available_robots, part, pose):
    # FIXME For now return a fixed team for each part
    if part.GetName() == 'WingSkin':
        # Choose the first robot
        team = Team([available_robots[0]])
    elif part.GetName() == 'Fastener':
        # Choose the third robot
        team = Team([available_robots[2]])
    else:
        raise Exception('Unknown part name: %s'%(part.GetName()))
    return team

def MovePartTo(allrobots, part, pose):
    # Decide on which robots to use, form a team
    team = FormTeam(allrobots, part, pose)
    grasp = team.PlanGrasp(part, pose)
    team.Grasp(grasp, part)
    team.MoveGraspedPartTo(pose) 


#np.set_printoptions(suppress=True)
#np.set_printoptions(precision=4)

env = orpy.Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/youbot/ros_stacks/drl-youbot/openrave/models/environments/fouryoubots.env.xml') 
#env.SetDebugLevel(orpy.DebugLevel.Debug);

wingskin = env.GetKinBody('WingSkin')
#fastener = env.GetKinBody('Fastener')
allrobots = env.GetRobots()

# Decide on assembly pose.
assembly_pose = GetAssemblyPose()
#wingskin.SetTransform(assembly_pose)

MovePartTo(allrobots, wingskin, assembly_pose)

# FIXME Third robot grasps its part and moves to assembly pose
# MovePartTo(allrobots, fastener, fastener_pose)
# FIXME Insert part

ipshell()

robot = env.GetRobots()[0]
manip = robot.GetManipulators()[0]

# Load the IK database.
with robot.GetEnv():
    robot.SetActiveManipulator(manip)
    manip.ik_database = orpy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=orpy.IkParameterization.Type.TranslationDirection5D)
    if not manip.ik_database.load():
        print 'Generating IK database for %s.'%manip.GetName()
        manip.ik_database.autogenerate()

target = np.array([ 0.42,  0,  0.13])
#target = np.array([ 0.39,  -0.06220808,  0.2342928])
#target = np.array([ 0.39,  0.0,  0.2342928])
direction = np.array([-1,0,0])
zstep = 0.001
prevtarget = None

# move up to find highest point with an ik.
while True:
    solution = manip.FindIKSolution(orpy.IkParameterization(orpy.Ray(target,direction),orpy.IkParameterization.Type.TranslationDirection5D),orpy.IkFilterOptions.CheckEnvCollisions)
    if solution is None:
        print 'highest: ',prevtarget
        target = prevtarget.copy()
        break
    else: 
        prevtarget = target.copy()
        target[2] = target[2] + zstep

path = []
while True:
    solution = manip.FindIKSolution(orpy.IkParameterization(orpy.Ray(target,direction),orpy.IkParameterization.Type.TranslationDirection5D),orpy.IkFilterOptions.CheckEnvCollisions)
    if solution is None:
        print 'lowest: ',prevtarget
        break
    else: 
        path.append(solution)
        prevtarget = target.copy()
        target[2] = target[2] - zstep

for q in path:
    robot.SetDOFValues(q,manip.GetArmIndices())
    print convert_to_real_youbot_joint_values(q)
    time.sleep(0.2)


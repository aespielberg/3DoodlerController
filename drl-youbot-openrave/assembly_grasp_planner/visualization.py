#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import time
import sys
import pickle
import random
import copy
import IPython
from itertools import izip,product,combinations,chain

import openravepy as orpy
import youbotpy
from youbotpy import youbotik as yik
import GraspGenerator
import AnytimePlanner
import CollisionConstraints
import copy

#from chairenv2 import *
#from pictureenv import *
#from wingdemoenv2 import *
#from chair_human_help import *
from chairenv3new import *

BACKUP_AMT = 0.05

youbotenv = youbotpy.YoubotEnv(sim=True,viewer=True,env_xml=envfile, \
                               youbot_names=all_robot_names)
env = youbotenv.env
youbots = youbotenv.youbots
for name in youbots:
    youbots[name].SetTransform(robot_start_poses[name])
    youbotenv.MoveGripper(name,0.01,0.01) # open grippers

yik.init(youbots[youbots.keys()[0]]) #   does not matter which robot
                                     # we use to initialize since 
                                     # all youbots have same kinematics.
grasp_generator = GraspGenerator.GraspGenerator(env)

for part in all_parts:
    part.body = env.GetKinBody(part.name)
    part.SetGrasps(grasp_generator.GeneratePartGrasps(part))

robot_base_homes = {}
for r in all_robot_names:
    robot_base_homes[r] = youbots[r].GetTransform()

planners = {}
for r in all_robot_names:
    planners[r] = orpy.interfaces.BaseManipulation(youbots[r],plannername='BiRRT')

anytime_planner = AnytimePlanner.AnytimePlanner()

# chairenv op_pose
op_pose = np.eye(4)
# frameenv op_pose
#op_pose[2,3] = 0.35 
# wingdemoenv2 op_pose
#op_pose = np.array([[ 0.7009, -0.7133,  0.    , -1.2478],
#                    [ 0.7133,  0.7009,  0.    ,  0.3399],
#                    [ 0.    ,  0.    ,  1.    ,  0.13  ],
#                    [ 0.    ,  0.    ,  0.    ,  1.    ]])

variables,asm_op_variables = anytime_planner.ComputeVariables(assembly_operations)
values = anytime_planner.ComputeAllValues(env,yik,variables,op_pose)

#IPython.embed()
#assembly_operations[0].Visualize(op_pose)
#grasp_generator.Visualize(assembly_operations[0])
#IPython.embed()

#youbotenv.ReplaceWithHiResYoubot('drc1')
youbots['drc2'].SetVisible(False)
youbots['drc3'].SetVisible(False)
# set camera pose
cam_transform = np.array([[-0.93931651, -0.14691819,  0.30999926, -2.0035615 ],
                          [-0.336758  ,  0.56720091, -0.75158312,  1.423581  ],
                          [-0.06541063, -0.81036916, -0.58225705,  0.99968624],
                          [ 0.        ,  0.        ,  0.        ,  1.        ]])
env.GetViewer().SetCamera(cam_transform)

for v in variables:
    print str(v)

c = ''
while not (c == 'q'):
    if c in values.keys():
        for v in variables:
            if str(v) == c:
                if not (type(v.parent_operation) is Part):
                    v.parent_operation.Visualize(op_pose)
                break
        time.sleep(3.0)
        for config in values[c]:
            #config.Visualize(env,'drc1_hires')
            config.Visualize(env,'drc1')
            time.sleep(0.05)
    c = raw_input('which var?')

IPython.embed()



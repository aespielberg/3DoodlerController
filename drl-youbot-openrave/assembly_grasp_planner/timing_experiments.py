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
import NaiveOptimalPlanner

#from chairenv2 import *
#experiment_name='chairenv2'
#from pictureenv import *
#experiment_name='pictureenv'
from randomarrayenv import *
experiment_name='randomarrayenv3'
#from wingdemoenv import *
#experiment_name='wingdemoenv'

youbotenv = youbotpy.YoubotEnv(sim=True,viewer=False,env_xml=envfile, \
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

#IPython.embed()

anytime_planner = AnytimePlanner.AnytimePlanner()
naive_planner = NaiveOptimalPlanner.NaiveOptimalPlanner()

op_pose = np.eye(4)
#op_pose[2,3] = 0.35
op_pose[0, 3] = -100
op_pose[1, 3] = -100
#raw_input('hit enter to start planning')
local_results = []
optimal_results = []
total_times_local = []
total_times_optimal = []
for i in range(1):
    if i < 1:
        local = True
        time_budget_minutes = 20.0
    else:
        local = True
        time_budget_minutes = 20.0
    total_time = 0
    start_time = time.time()
    #IPython.embed()
    if local:
        min_regrasp_assignment,min_regrasp_constraints,variables,asm_op_variables = anytime_planner.Plan(env,yik,op_pose,assembly_operations,all_robot_names,time_budget_minutes)
        if min_regrasp_assignment is None:
            local_results.append((None,None,None))
            total_times_local.append(time.time() - start_time)
            print 'one local failure recorded.'
        else:
            record_n_transfers,record_times = anytime_planner.GetLastRunRecords()
            local_results.append((record_n_transfers,record_times,min_regrasp_assignment))
            total_times_local.append(time.time() - start_time)
            print 'one local result recorded.'
    else: # optimal solution
        min_regrasp_assignment,min_regrasp_constraints,variables,asm_op_variables = naive_planner.Plan(env,yik,op_pose,assembly_operations,all_robot_names,time_budget_minutes)
        if min_regrasp_assignment is None:
            optimal_results.append((None,None,None))
            total_times_optimal.append(time.time() - start_time)
            print 'one optimal failure recorded.'
        else:
            record_n_transfers,record_times = naive_planner.GetLastRunRecords()
            optimal_results.append((record_n_transfers,record_times,min_regrasp_assignment))
            total_times_optimal.append(time.time() - start_time)
            print 'one optimal result recorded.'

with open('experiment_results/results_'+experiment_name+'.txt','a') as f:
    pickle.dump(local_results,f)
    pickle.dump(total_times_local,f)
    pickle.dump(optimal_results,f)
    pickle.dump(total_times_optimal,f)

#IPython.embed()

orpy.RaveDestroy()



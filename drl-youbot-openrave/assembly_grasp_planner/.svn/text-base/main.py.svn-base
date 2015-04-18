#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import pickle
import random
import copy
import IPython
from itertools import izip,product

import openravepy as orpy

import youbotpy
from youbotpy import youbotik as yik

import GraspGenerator
import Assembly

from chairenv import *
# from wingdemoenv import *

doconfigplan = False

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

all_part_poses = {} 
all_robot_poses = {}
for part in all_parts:
    all_part_poses[part.name] = part.body.GetTransform()
for rob in all_robot_names:
    all_robot_poses[rob] = env.GetRobot(rob).GetTransform()

def FindGrasp(assembly_operations,prev_robot_configs):
    if len(assembly_operations) == 0:
        print 'done'
        return []
    existing_robot_part_grasps = []
    for rc in prev_robot_configs:
        for asm in assembly_operations[-1].assembly_list:
            #print 'Checking if assembly ',asm.name,' includes ',rc[1].part_grasp.part_name
            if asm.Includes(rc[1].part_grasp.part_name):
                existing_robot_part_grasps.append((rc[1].robot_name,rc[1].part_grasp))
                #print 'yes, breaking'
                break # only one for now
        if len(existing_robot_part_grasps) > 0:
            break # only one for now
    #for rc in prev_robot_configs:
    #    if assembly_operations[-1].assembly_list[0].Includes(rc[1].part_grasp.part_name):
    #        existing_robot_part_grasps.append((rc[1].robot_name,rc[1].part_grasp))
    #        break # only one for now
    while True:
        print 'Planning op: ', assembly_operations[-1].name
        if len(existing_robot_part_grasps) > 0:
            print 'existing grasp: ', existing_robot_part_grasps[0][1].part_name
        robot_configs = assembly_operations[-1].PlanRobotGraspConfigs(env,yik,all_robot_names,all_object_names,np.eye(4),
                                                                      randomize=True,
                                                                      existing_robot_part_grasps=existing_robot_part_grasps)
        if robot_configs is None:
            print 'no solution found'
            return None
        robot_config_plan = FindGrasp(assembly_operations[:-1],robot_configs) 
        if robot_config_plan is None:
            continue # TODO This is stupid, backtrack all the way upto where the part is grasped.
        robot_config_plan.append(robot_configs)
        return robot_config_plan

for assembly_operation in assembly_operations:
    for assembly in assembly_operation.assembly_list:
        grasp_generator.GenerateAssemblyGrasps(assembly)

robot_config_plan=FindGrasp(assembly_operations,[])

raw_input('hit enter to visualize')

def VisualizePlan(env,plan,assembly_operations,all_robot_names,all_object_names):
    away = np.eye(4)
    away[0,3]=1000.0
    for robot_configs,assembly_operation in izip(plan,assembly_operations):
        # Move robots and parts away.
        for r in all_robot_names:
            env.GetRobot(r).SetTransform(away)
        for o in all_object_names:
            env.GetKinBody(o).SetTransform(away)
        for rc in robot_configs:
            rc[1].Visualize(env)
        assembly_operation.Visualize(np.eye(4))
        raw_input('Hit enter for next step.')

VisualizePlan(env,robot_config_plan,assembly_operations,all_robot_names,all_object_names)
IPython.embed()


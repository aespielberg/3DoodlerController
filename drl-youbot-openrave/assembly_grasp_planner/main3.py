#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import pickle
import random
import copy
import IPython
from itertools import izip,product,combinations,chain

import openravepy as orpy

import youbotpy
from youbotpy import youbotik as yik

import GraspGenerator
#import Assembly
from AssemblyOperation import PartGrasp,Part
import AssemblyGraspCSP
import CollisionConstraints
import BusRide

from chairenv import *
#from pictureenv import *
#from wingdemoenv import *

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

#IPython.embed()

def ComputeExtendedBusRide(assembly_operations,busride):
    # TODO return None if fastener is in the busride 
    extended_busride = {}
    for asm_op in assembly_operations:
        extended_busride[asm_op.name] = []
        for child_asm_op in asm_op.assembly_op_list:
            extended_busride[asm_op.name].extend(busride[child_asm_op.name])
    return extended_busride

def ComputeValuesForVariable(var,env,robot_name,op_pose):
    return var.parent_operation.GetRobotGraspConfigs(env,yik,robot_name,var.assembly,op_pose)

def GenerateCollisionConstraints(env,all_robot_names,asm_op_variables):
    constraints = []
    for asm_to_vars in asm_op_variables.values():
        for var1,var2 in combinations(chain.from_iterable(asm_to_vars.values()),2):
            c = CollisionConstraints.CollisionConstraints2(env,all_robot_names,var1,var2)
            constraints.append(c)
    return constraints

def GenerateBusRideConstraints(extended_busride,asm_op_variables):
    constraints = []
    for op in assembly_operations:
        counts_asm = {}
        counts_asm_subasm = {}
        for (part_name,count) in extended_busride[op.name]:
            done=False
            for asm in op.assembly_op_list:
                if (not (type(asm) is Part)) and (asm.IncludesPart(part_name)):
                    for subasm in asm.assembly_op_list:
                        if subasm.IncludesPart(part_name):
                            if not (asm.name in counts_asm.keys()):
                                counts_asm[asm.name] = 0
                            var1 = asm_op_variables[op.name][asm.name][counts_asm[asm.name]]
                            counts_asm[asm.name] += 1
                            if not (asm.name in counts_asm_subasm.keys()):
                                counts_asm_subasm[asm.name] = {}
                            if not (subasm.name in counts_asm_subasm[asm.name].keys()):
                                counts_asm_subasm[asm.name][subasm.name] = 0
                            var2 = asm_op_variables[asm.name][subasm.name][counts_asm_subasm[asm.name][subasm.name]]
                            counts_asm_subasm[asm.name][subasm.name] += 1
                            c = CollisionConstraints.BusrideConstraint(var1,var2)
                            constraints.append(c)
                            done=True
                            break 
                    if done: 
                        break
    return constraints

op_pose = np.eye(4)
#op_pose[2,3] = 0.35
# Generate variables (one variable for every grasp)
variables = []
asm_op_variables = {}
for parent_asm_op in assembly_operations:
    asm_op_variables[parent_asm_op.name] = {}
    for asm_op in parent_asm_op.assembly_op_list:
        asm_op_variables[parent_asm_op.name][asm_op.name] = [] 
        for i in range(asm_op.robot_count):
            var = AssemblyGraspCSP.AssemblyCSPVariable2(parent_asm_op,asm_op,i)
            variables.append(var)
            asm_op_variables[parent_asm_op.name][asm_op.name].append(var)
# Generate value set for each variable
values = {}
for var in variables:
    values[str(var)] = ComputeValuesForVariable(var,env,'drc1',op_pose)
    #print str(var),' has ',len(values[str(var)]),' values'
# Generate collision constraints
collision_constraints = GenerateCollisionConstraints(env,all_robot_names,asm_op_variables)
busride_generator = BusRide.BusRide(assembly_operations)
busride = None
plan = None
while True:
    print 'getting new busride' # get busride. a busride is a dict from asm_op names to list of (part,count) pairs
    busride = busride_generator.get_next_busride()
    busride = busride_generator.get_next_busride()
    busride = busride_generator.get_next_busride()
    if busride is None:
        print 'All busrides tried.'
        break
    extended_busride = ComputeExtendedBusRide(assembly_operations,busride)
    if extended_busride is None: # This is just a hackway to filter out busrides with fasteners.
        continue
    busride_constraints = GenerateBusRideConstraints(extended_busride,asm_op_variables)
    constraints = []
    constraints.extend(collision_constraints)
    constraints.extend(busride_constraints)
    # backtracking search
    raw_input('hit enter to run csp')
    plan = AssemblyGraspCSP.BacktrackingSearch2(variables,values,constraints)
    if not (plan is None):
        break

#IPython.embed()
raw_input('hit enter to visualize')

def VisualizePlan(env,plan,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose):
    away = np.eye(4)
    away[0,3]=1000.0
    for assembly_operation in assembly_operations:
        # Move robots and parts away.
        for r in all_robot_names:
            env.GetRobot(r).SetTransform(away)
        for o in all_object_names:
            env.GetKinBody(o).SetTransform(away)
        rem_robots = copy.copy(all_robot_names)
        op_vars = asm_op_variables[assembly_operation.name]
        #IPython.embed()
        for var in chain.from_iterable(op_vars.values()):
            plan[str(var)].Visualize(env,rem_robots[-1])
            del rem_robots[-1]
        assembly_operation.Visualize(op_pose)
        raw_input('Hit enter for next step.')

VisualizePlan(env,plan,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose)
IPython.embed()


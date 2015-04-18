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
#import Assembly
from AssemblyOperation import PartGrasp
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

# For all assembly operations create the robotconfigs per part per grasp
# [partname] = list of dicts {asmopname:robotconfig} s.t. in each dict the
# robotconfigs correspond to the same grasp of the part
def GetAssemblyOperationsGrasps(env,all_parts,assembly_operations,all_robot_names,op_pose):
    asm_ops_grasps = {}
    for part in all_parts:
        asm_ops_grasps[part.name] = []
        for g in part.grasps:
            asm_op_configs = [] 
            for asm_op in assembly_operations:
                if asm_op.IncludesPart(part.name):
                    child_asm_op,child_asm_op_pose = asm_op.GetPartAssemblyAndAssemblyPose(part.name)
                    configs_in_asm_op = asm_op.GetNonCollidingRobotGraspConfigs(env,
                                                                                yik,
                                                                                all_robot_names[0],
                                                                                PartGrasp(part.name,g),
                                                                                child_asm_op,child_asm_op_pose,op_pose)
                    if (configs_in_asm_op is None) or (len(configs_in_asm_op) == 0):
                        asm_op_configs.append([None])
                    else:
                        configs_in_asm_op.append(None) # This is so that we can filter out unnecessary configs later using a busride.
                        asm_op_configs.append(configs_in_asm_op)
                else:
                    asm_op_configs.append([None])
            all_config_combinations = []
            # compute all combinations of configs for different asm_ops where
            # each combination is a dict {asmopname:robotconfig}
            for configs in product(*asm_op_configs): 
                config_dict = {}
                for asm_op,config in izip(assembly_operations,configs):
                    config_dict[asm_op.name] = config
                all_config_combinations.append(config_dict)
            # Add the configs for grasp g
            asm_ops_grasps[part.name].extend(all_config_combinations)
    return asm_ops_grasps

def ComputeExtendedBusRide(assembly_operations,busride):
    # TODO return None if fastener is in the busride 
    extended_busride = {}
    for asm_op in assembly_operations:
        extended_busride[asm_op.name] = []
        for child_asm_op in asm_op.assembly_op_list:
            extended_busride[asm_op.name].extend(busride[child_asm_op.name])
    return extended_busride

def ComputeValues(var,configs_for_part,assembly_operations,busride):
    values = []
    for config in configs_for_part:
        goodconfig = True
        for asm_op in assembly_operations:
            if (var.partname,var.count) in busride[asm_op.name]: # the part needs to be grasped in this asm_op for this busride
                if config[asm_op.name] is None: # this grasp does not work in this asm_op
                    goodconfig = False
                    break
            else:
                if not (config[asm_op.name] is None):
                    # TODO Instead of filtering like this just generate the product space of values for every new busride.
                    goodconfig = False
                    break
        if goodconfig:
            values.append(config)
    return values


print 'generating grasps'
op_pose = np.eye(4)
#op_pose[2,3] = 0.35
#op_pose[2,3] = 0.45
assembly_grasps_per_part = GetAssemblyOperationsGrasps(env,all_parts,assembly_operations,all_robot_names,op_pose) 
#IPython.embed() # XXX: Use this IPython shell to visualize grasps.
print 'before creating busride gen'
busride_generator = BusRide.BusRide(assembly_operations)
print 'created busride gen'
busride = None
plan = None
t1 = time.time()
transfer_schemes = 0
vals_assigned = 0
while True:
    # get busride. a busride is a dict from asm_op names to list of (part,count) pairs
    print 'getting new busride'
    transfer_schemes += 1
    busride = busride_generator.get_next_busride()
    busride = busride_generator.get_next_busride()
    busride = busride_generator.get_next_busride()
    #print busride
    IPython.embed()
    if busride is None:
        print 'All busrides tried.'
        break
    extended_busride = ComputeExtendedBusRide(assembly_operations,busride)
    if extended_busride is None:
        # This is just a hackway to filter out busrides with fasteners.
        continue
    # extract CSP variable list for this busride 
    variables = []
    for asm_op in assembly_operations:
        for partname_count_pair in extended_busride[asm_op.name]:  
            var = AssemblyGraspCSP.AssemblyCSPVariable(partname_count_pair[0],partname_count_pair[1])
            if not (var in variables):
                variables.append(var)
    # create value list for the busride using part_names as indexes into allconfigs
    values = {} 
    for var in variables:
        values[str(var)] = ComputeValues(var,assembly_grasps_per_part[var.partname],assembly_operations,extended_busride)
        vals_assigned += 1
    # create constraints for this CSP
    constraints = CollisionConstraints.CollisionConstraints(env,assembly_operations,all_robot_names,extended_busride)
    # call CSP
    #raw_input('hit enter to run csp')
    plan = AssemblyGraspCSP.BacktrackingSearch(variables,values,constraints)
    if not (plan is None):
        break

#IPython.embed()
print time.time() - t1
print transfer_schemes
print vals_assigned
raw_input('hit enter to visualize')

def VisualizePlan(env,plan,extended_busride,assembly_operations,all_robot_names,all_object_names, op_pose):
    away = np.eye(4)
    away[0,3]=1000.0
    for assembly_operation in assembly_operations:
        # Move robots and parts away.
        for r in all_robot_names:
            env.GetRobot(r).SetTransform(away)
        for o in all_object_names:
            env.GetKinBody(o).SetTransform(away)
        rem_robots = copy.copy(all_robot_names)
        for var in extended_busride[assembly_operation.name]:
            for pairs in plan:
                if var == (pairs[0].partname,pairs[0].count): 
                    assembly_grasp_for_part = pairs[1]
                    break
            if not (assembly_grasp_for_part[assembly_operation.name] is None):
                assembly_grasp_for_part[assembly_operation.name].Visualize(env,rem_robots[-1])
                del rem_robots[-1]
        assembly_operation.Visualize(op_pose)
        raw_input('Hit enter for next step.')

VisualizePlan(env,plan,extended_busride,assembly_operations,all_robot_names,all_object_names,op_pose)
IPython.embed()



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
    constraints_per_op = {}
    constraints = []
    for asm_to_vars,op_name in izip(asm_op_variables.values(),asm_op_variables.keys()):
        constraints_per_op[op_name] = []
        for var1,var2 in combinations(chain.from_iterable(asm_to_vars.values()),2):
            c = CollisionConstraints.CollisionConstraints2(env,all_robot_names,var1,var2)
            constraints.append(c)
            constraints_per_op[op_name].append(c)
    return constraints,constraints_per_op

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

def ContainsDuplicateVar2(c_list):
    for i,c in enumerate(c_list):
        for c_other in c_list[i+1:]:
            if c.var2 == c_other.var2:
                return True
    return False

def GetAllPossibleBusrideConstraintsForVars(variables,asm_op_variables):
    var_constraints = []
    for var in variables:
        cur_var_constraints = []
        for subasm in var.assembly.assembly_op_list:
            if not (type(subasm) is Part and not subasm.is_transferable):
                for var2 in asm_op_variables[var.assembly.name][subasm.name]: # In case there are more than one grasp of this subasm, create a new constraint for each.
                    c = CollisionConstraints.BusrideConstraint(var,var2)
                    cur_var_constraints.append(c)
        var_constraints.append(cur_var_constraints)
    constraints = [c for c in product(*var_constraints)]
    # Filter constraints out if they include link to same var2. (necessary for multi-grasp of same assembly).
    constraints = [c for c in constraints if not ContainsDuplicateVar2(c)]
    #IPython.embed()
    return constraints

# Returns the list consisting of sets of N busride constraints.
def GetNBusrideConstraintsSpace(n,assembly_operations,variables):
    space = []
    # filter out trivial variables
    non_trivial_vars = [var for var in variables if not (type(var.assembly) is Part)]
    for n_comb in combinations(non_trivial_vars,n):
        new_space = GetAllPossibleBusrideConstraintsForVars(n_comb,asm_op_variables)
        space.extend(new_space)
    return space

def IncludesFailed(current,failed_list):
    for failed in failed_list:
        if set(failed) < set(current):
            return True
    return False
 
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
pretimes = []
pretime_start = time.time()
values = {}
for var in variables:
    values[str(var)] = ComputeValuesForVariable(var,env,'drc1',op_pose)
    #print str(var),' has ',len(values[str(var)]),' values'
# Generate collision constraints
collision_constraints,collision_constraints_per_op = GenerateCollisionConstraints(env,all_robot_names,asm_op_variables)
pretimes.append(time.time()-pretime_start)
#raw_input('hit enter to start planning')
time_budget_minutes = 10.0
local_results = []
optimal_results = []
for i in range(1):
    if i < 1:
        local = True
    else:
        local = False
    start_time = time.time()
    plan = {}
    if local:
        assignment_per_op = {}
        no_soln = False
        for op in assembly_operations:
            t = time.time()
            print 'planning for op ',op.name
            op_variables = [var for var in chain.from_iterable(asm_op_variables[op.name].values()) ]
            op_constraints = collision_constraints_per_op[op.name]
            assignment_per_op[op.name] = AssemblyGraspCSP.BacktrackingSearch2(op_variables,values,op_constraints,start_time+time_budget_minutes*60.0)
            print time.time()-t,'sec passed.'
            if assignment_per_op[op.name] is None:
                print 'no solution for op: ',op.name
                no_soln = True
                break
            else:
                print 'op ',op.name,' planned.'
        
        if not no_soln:
            for op in assembly_operations:
                for var_str in assignment_per_op[op.name].keys():
                    plan[var_str] = assignment_per_op[op.name][var_str]
        
            constraints = []
            constraints.extend(collision_constraints)
            
            print('Found solution without busride constraints. Now trying to fix regrasps.')
            #IPython.embed()
            
            all_regrasp_plan = copy.copy(plan)
            #all_regrasp_plan = {}
            # random initial assignment
            #for v in variables:
            #    all_regrasp_plan[str(v)] = random.choice(values[str(v)])
            all_regrasp_constraints = copy.copy(constraints)
            non_trivial_vars = [var for var in variables if not (type(var.assembly) is Part)]
            max_n_regrasps = len(non_trivial_vars)
            min_regrasp_assignment = all_regrasp_plan
            min_regrasp_constraints = None
            min_regrasp_c_list = []
            min_regrasp_n = 0
            failed_list = []
            record_n_transfers = [0]
            record_times = [time.time()-start_time]
            must_exit=False
            for n_transfers in range(1,max_n_regrasps+1):
                if time.time()-start_time > time_budget_minutes*60.0 or must_exit:
                    break
                c_lists = GetNBusrideConstraintsSpace(n_transfers,assembly_operations,variables)
                random.shuffle(c_lists) 
                semi_ordered_clists = []
                for c_list in c_lists:
                    if set(min_regrasp_c_list) < set(c_list):
                        semi_ordered_clists.insert(0,c_list)
                    else:
                        semi_ordered_clists.append(c_list)
                c_lists = semi_ordered_clists
                for c_list in c_lists:
                    if time.time()-start_time > time_budget_minutes*60.0:
                        break
                    #if not (IncludesFailed(c_list,failed_list)): # pruning
                    if True: # pruning
                        new_constraints = copy.copy(all_regrasp_constraints)
                        new_constraints.extend(c_list)
                        ## TODO HACK TO SPECIFY A T-CONST. STARTS
                        #var1 = asm_op_variables['rightside_and_back']['back'][0]
                        #var2 = asm_op_variables['rightside_and_back_and_seat']['rightside_and_back'][0]
                        #c = CollisionConstraints.BusrideConstraint(var1,var2)
                        #new_constraints.append(c)
                        ## TODO HACK TO SPECIFY A T-CONST. ENDS
                        #new_assignment = copy.copy(all_regrasp_plan)
                        new_assignment = copy.copy(min_regrasp_assignment)
                        print 'one minconflict attempt. n: ',n_transfers
                        success = AssemblyGraspCSP.MinConflict(variables,values,new_constraints,new_assignment,max_steps=5*n_transfers)
                        print 'minconflict attempt done!'
                        if success:
                            record_n_transfers.append(n_transfers) 
                            record_times.append(time.time()-start_time)
                            print 'solution with ',n_transfers,' busride constraints found!'
                            min_regrasp_assignment = new_assignment
                            min_regrasp_constraints = new_constraints
                            min_regrasp_c_list = c_list
                            min_regrasp_n = n_transfers
                            break
                        else:
                            failed_list.append(c_list)
                    else:
                        print 'constraint list pruned.'
            local_results.append((record_n_transfers,record_times))
            print 'one local result recorded.'
        else:
            local_results.append((None,None))
            print 'one local failure recorded.'
    else: # optimal solution
        ##constraints = []
        #constraints.extend(collision_constraints)
        non_trivial_vars = [var for var in variables if not (type(var.assembly) is Part)]
        max_n_regrasps = len(non_trivial_vars)
        must_exit = False
        for n_transfers in range(max_n_regrasps,-1,-1): # backwards until 0
            if time.time()-start_time > time_budget_minutes*60.0 or must_exit:
                break
            c_lists = GetNBusrideConstraintsSpace(n_transfers,assembly_operations,variables)
            random.shuffle(c_lists) 
            for c_list in c_lists:
                if time.time()-start_time > time_budget_minutes*60.0:
                    break
                new_constraints = copy.copy(collision_constraints)
                new_constraints.extend(c_list)
                print 'one backtrack search attempt. n: ',n_transfers
                optimal_assignment = AssemblyGraspCSP.BacktrackingSearch2(variables,values,new_constraints,start_time+time_budget_minutes*60.0)
                print 'backtrack search attempt done!'
                if not (optimal_assignment is None): 
                    print 'optimal solution found!'
                    record_times = [time.time()-start_time]
                    record_n_transfers = [n_transfers]
                    must_exit = True
                    min_regrasp_assignment = optimal_assignment
                    break
        if optimal_assignment is None:
            optimal_results.append((None,None))
            print 'one optimal failure recorded.'
        else:
            optimal_results.append((record_n_transfers,record_times))
            print 'one optimal result recorded.'

#raw_input('hit enter to visualize')

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

#if not (min_regrasp_assignment is None):
#    VisualizePlan(env,min_regrasp_assignment,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose)

IPython.embed()
 

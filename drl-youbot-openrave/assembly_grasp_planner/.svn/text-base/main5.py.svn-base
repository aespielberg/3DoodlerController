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
#import Assembly
from AssemblyOperation import PartGrasp,Part
import AssemblyGraspCSP
import CollisionConstraints
import BusRide

from chairenv2 import *
#from pictureenv import *
#from wingdemoenv import *

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

robot_base_homes = {}
for r in all_robot_names:
    robot_base_homes[r] = youbots[r].GetTransform()

planners = {}
for r in all_robot_names:
    planners[r] = orpy.interfaces.BaseManipulation(youbots[r],plannername='BiRRT')

#IPython.embed()

def ComputeValuesForVariable(v,env,robot_name,op_pose):
    return v.parent_operation.GetRobotGraspConfigs(env,yik,robot_name,v.assembly,op_pose)

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

def ContainsDuplicateVar2(c_list):
    for i,c in enumerate(c_list):
        for c_other in c_list[i+1:]:
            if c.var2 == c_other.var2:
                return True
    return False

def GetAllPossibleBusrideConstraintsForVars(variables,asm_op_variables):
    var_constraints = []
    for v in variables:
        if type(v.parent_operation) is Part:
            continue
        cur_var_constraints = []
        for subasm in v.assembly.assembly_op_list:
            for var2 in asm_op_variables[v.assembly.name][subasm.name]: # In case there are more than one grasp of this subasm, create a new constraint for each.
                c = CollisionConstraints.BusrideConstraint(v,var2)
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
    non_trivial_vars = [v for v in variables if not (type(v.parent_operation) is Part)]
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
            v = AssemblyGraspCSP.AssemblyCSPVariable2(parent_asm_op,asm_op,i)
            variables.append(v)
            asm_op_variables[parent_asm_op.name][asm_op.name].append(v)
# Generate value set for each variable
pretimes = []
pretime_start = time.time()
values = {}
for v in variables:
    this_op_pose = op_pose
    if type(v.parent_operation) is Part:
        this_op_pose = env.GetKinBody(v.parent_operation.name).GetTransform() # Use current transform of part in openrave world.
    values[str(v)] = ComputeValuesForVariable(v,env,'drc1',this_op_pose)
    #print str(v),' has ',len(values[str(v)]),' values'
# Generate collision constraints
collision_constraints,collision_constraints_per_op = GenerateCollisionConstraints(env,all_robot_names,asm_op_variables)
pretimes.append(time.time()-pretime_start)
#raw_input('hit enter to start planning')
time_budget_minutes = 30.0
local_results = []
optimal_results = []
for i in range(2):
    if i < 1:
        local = False
    else:
        local = True
    start_time = time.time()
    plan = {}
    if local:
        assignment_per_op = {}
        no_soln = False
        for op in assembly_operations:
            t = time.time()
            print 'planning for op ',op.name
            op_variables = [v for v in chain.from_iterable(asm_op_variables[op.name].values()) ]
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
            non_trivial_vars = [v for v in variables if not (type(v.parent_operation) is Part)]
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
                        new_assignment = copy.copy(min_regrasp_assignment)
                        print 'one minconflict attempt. n: ',n_transfers
                        success = AssemblyGraspCSP.MinConflict(variables,values,new_constraints,new_assignment,max_steps=10*n_transfers)
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
        non_trivial_vars = [v for v in variables if not (type(v.parent_operation) is Part)]
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
                    min_regrasp_constraints = new_constraints
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
    orig_part_poses = {}
    for o in all_object_names:
        orig_part_poses[o] = env.GetKinBody(o).GetTransform()
    for assembly_operation in assembly_operations:
        # Move robots and parts away.
        for r in all_robot_names:
            env.GetRobot(r).SetTransform(away)
        for o in all_object_names:
            env.GetKinBody(o).SetTransform(away)
        rem_robots = copy.copy(all_robot_names)
        op_vars = asm_op_variables[assembly_operation.name]
        #IPython.embed()
        for v in chain.from_iterable(op_vars.values()):
            plan[str(v)].Visualize(env,rem_robots[-1])
            del rem_robots[-1]
        this_op_pose = op_pose
        if type(assembly_operation) is Part:
            this_op_pose = orig_part_poses[assembly_operation.name] # Use current transform of part in openrave world.
        assembly_operation.Visualize(this_op_pose)
        raw_input('Hit enter for next step.')

#if not (min_regrasp_assignment is None):
#    VisualizePlan(env,min_regrasp_assignment,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose)

def ChooseClosest(pose,robot_names,youbots):
    min_dist_sqr = sys.float_info.max
    min_dist_robot_name = ''
    for n in robot_names:
        diff = youbots[n].GetTransform()[:3,3]-pose[:3,3]
        dist_sqr = diff.x**2 + diff.y**2
        if dist_sqr <= min_dist_sqr:
            min_dist_robot_name = n
            min_dist_sqr = dist_sqr
    return min_dist_robot_name

def IsTransferTarget(v,constraints):
    for c in constraints:
        if type(c) is CollisionConstraints.BusrideConstraint:
            if v == c.var1:
                print 'Transfer target! ',v.assembly.name
                return True
    print 'NOT Transfer target! ',v.assembly.name
    return False

def IsTransferConstraint(source,target,constraints):
    for c in constraints:
        if type(c) is CollisionConstraints.BusrideConstraint:
            if target == c.var1 and source == c.var2:
                print 'Transfer constraint! s:, ',source.assembly.name,', t: ',target.assembly.name
                return True
    print 'NOT! Transfer constraint! s:, ',source.assembly.name,', t: ',target.assembly.name
    return False

def ChooseRobot(robots):
    return random.choice(robots)

def AssignRobotsToOperations(variables,constraints,asm_op_variables,all_robot_names,assembly_operations):
    # grasp, transfer, regrasp, transfer_release, regrasp_release
    robot_ops_for_vars = {}
    for v in variables:
        if type(v.parent_operation) is Part:
            robot_ops_for_vars[v] = 'grasp'
        elif IsTransferTarget(v,constraints):
            robot_ops_for_vars[v] = 'transfer_release'
        else:
            robot_ops_for_vars[v] = 'regrasp_release'
    ## DEBUG
    #print '1:'
    #for v in variables:
    #    print 'var: p:',v.parent_operation.name, ', a: ', v.assembly.name, ', op: ',robot_ops_for_vars[v]
        
    var_connections = {} # mapping from one var to the prev var sharing the same robot
    for v in variables:
        #if type(v.parent_operation) is Part:
        if robot_ops_for_vars[v] == 'grasp':
            continue
        elif robot_ops_for_vars[v] == 'regrasp' or robot_ops_for_vars[v] == 'regrasp_release':
            if not (type(v.assembly) is Part):
                transferable_subasm_list = [x for x in v.assembly.assembly_op_list if (not ((type(x) is Part) and (not x.is_transferable)))]
            else:
                transferable_subasm_list = copy.copy(v.assembly.assembly_op_list)
            subasm = random.choice(transferable_subasm_list)
            choice_var = asm_op_variables[v.assembly.name][subasm.name][0] # Just takes the first grasp if there are multiple. Should not matter for regrasp.
        else: # 'transfer' or 'transfer_release'
            subasm_variables = [v2 for v2 in chain.from_iterable(asm_op_variables[v.assembly.name].values()) ]
            for subasm_v in subasm_variables:
                if IsTransferConstraint(source=subasm_v,target=v,constraints=constraints):
                    choice_var = subasm_v
                    break
        if robot_ops_for_vars[choice_var] == 'transfer_release':
            robot_ops_for_vars[choice_var] = 'transfer'
        elif robot_ops_for_vars[choice_var] == 'regrasp_release': 
            robot_ops_for_vars[choice_var] = 'regrasp'
        var_connections[v] = choice_var

    ## DEBUG
    #print '2:'
    #for v in variables:
    #    print 'var: p:',v.parent_operation.name, ', a: ', v.assembly.name, ', op: ',robot_ops_for_vars[v]

    robots_for_vars = {}
    free_robots = copy.copy(all_robot_names)
    for op in assembly_operations:
        op_variables = [v for v in chain.from_iterable(asm_op_variables[op.name].values()) ]
        for v in op_variables:
            if robot_ops_for_vars[v] == 'grasp':
                rob = ChooseRobot(free_robots)
                robots_for_vars[v] = rob
                free_robots.remove(rob)
            else:
                robots_for_vars[v] = robots_for_vars[var_connections[v]]
                if (robot_ops_for_vars[v] == 'regrasp_release') or (robot_ops_for_vars[v] == 'transfer_release'):
                    free_robots.append(robots_for_vars[v])
    ## DEBUG
    #print '3:'
    #for v in variables:
    #    print 'var: p:',v.parent_operation.name, ', a: ', v.assembly.name, ', rob: ',robots_for_vars[v]

    return robots_for_vars,robot_ops_for_vars

def EnableGripperLinks(env,robot_name,enable):
    with env:
        rob = env.GetRobot(robot_name)
        rob.GetLink('leftgripper').Enable(enable)
        rob.GetLink('rightgripper').Enable(enable)
        rob.GetLink('link5').Enable(enable)

def MoveBaseTo(robot,goal,planner):
    xyyaw = np.array([goal[0,3],goal[1,3],np.arctan2(goal[1,0],goal[0,0])])
    current = robot.GetTransform()
    currentxyyaw = np.array([current[0,3],current[1,3],np.arctan2(current[1,0],current[0,0])])
    if np.linalg.norm(currentxyyaw-xyyaw) < 0.001:
        print robot.GetName(),' already at goal. Not moving.'
        return 
    with env:
        robot.SetActiveDOFs([],orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
        traj = planner.MoveActiveJoints(goal=xyyaw,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True)
        robot.GetController().SetPath(traj)
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def MoveArmTo(robot,goal,planner):
    with env:
        robot.SetActiveDOFs(range(5))
        planner.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2)
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def GrabAssembly(env,robot_name,assembly,grab=True):
    robot = env.GetRobot(robot_name)
    for part in assembly.GetPartList():
        if grab:
            robot.Grab(part.body)
        else:
            robot.Release(part.body)

def Regrasp(env,robot,assembly,new_pose_in_gripper):
    GrabAssembly(env,robot.GetName(),assembly,False)
    gripper_pose = robot.GetManipulators()[0].GetEndEffectorTransform()
    new_assembly_pose = np.dot(gripper_pose,new_pose_in_gripper)
    assembly.SetTransform(new_assembly_pose)
    GrabAssembly(env,robot.GetName(),assembly,True)

def Execute(env,youbots,robot_name,op_name,assembly,config,planners,robot_base_homes):
    EnableGripperLinks(env,robot_name,False)
    # Move home
    # MoveBaseTo(youbots[robot_name],robot_base_homes[robot_name],planners[robot_name]) 
    if op_name == 'regrasp' or op_name == 'regrasp_release':
        print 'REGRASP!'
        MoveArmTo(youbots[robot_name],np.array([0.,0.,0.,0.,0.]),planners[robot_name]) 
        new_ee_pose_in_assembly = np.dot(assembly.GetPartPoseInAssembly(config.part_grasp.part_name),config.part_grasp.grasp)
        regrasped_asm_in_gripper = np.linalg.inv(new_ee_pose_in_assembly)
        Regrasp(env,youbots[robot_name],assembly,regrasped_asm_in_gripper) 
    # Move to config
    MoveArmTo(youbots[robot_name],config.arm_config,planners[robot_name]) 
    MoveBaseTo(youbots[robot_name],config.base_config,planners[robot_name]) 
    EnableGripperLinks(env,robot_name,True)
    if op_name == 'grasp':
        GrabAssembly(env,robot_name,assembly,True)

def PrioritizeNontrivialVars(op_vars):
    ordered_op_vars = []
    parts = []
    for v in op_vars:
        if type(v.assembly) is Part:
            parts.append(v)
        else:
            ordered_op_vars.append(v)
    # HACK TO MOVE FASTENERS LAST
    parts2 = []
    for v in parts:
        if 'fastener' in v.assembly.name:
            parts2.append(v)
        else:
            parts2.insert(0,v)
    parts = parts2
    # HACK TO MOVE FASTENERS LAST - ENDS HERE
    ordered_op_vars.extend(parts)
    return ordered_op_vars

def ExecutePlan(env,youbots,plan,constraints,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose,planners,robot_base_homes):
    robots_for_vars,robot_ops_for_vars = AssignRobotsToOperations(variables,constraints,asm_op_variables,all_robot_names,assembly_operations)
    asm_to_robots = {}
    for op in assembly_operations:
        # order variables such that nontrivial ops get priority 
        op_vars = [v for v in chain.from_iterable(asm_op_variables[op.name].values()) ]
        ordered_op_vars = PrioritizeNontrivialVars(op_vars)
        for v in ordered_op_vars: # XXX this assumes one robot per assembly.
            #print str(v),' config: ',plan[str(v)].base_config
            Execute(env,youbots,robots_for_vars[v],robot_ops_for_vars[v],v.assembly,plan[str(v)],planners,robot_base_homes)
            #raw_input('Hit enter for next execution.')
        for v in ordered_op_vars:
            op_name = robot_ops_for_vars[v]
            r = robots_for_vars[v]
            if op_name == 'transfer_release' or op_name == 'regrasp_release':
                # Release assembly
                GrabAssembly(env,r,v.assembly,False)
                EnableGripperLinks(env,r,False)
                MoveBaseTo(youbots[r],robot_base_homes[r],planners[r]) 
                EnableGripperLinks(env,r,True)
                #raw_input('Hit enter for next execution.')
            elif op_name == 'transfer' or op_name == 'regrasp':
                GrabAssembly(env,r,v.assembly,False) # Release the subassembly 
                GrabAssembly(env,r,v.parent_operation,True) # Grab the full thing.

IPython.embed()

for c in min_regrasp_constraints:
    if type(c) is CollisionConstraints.BusrideConstraint:
        print 'Busride between: ',c.var1.assembly.name,' and ',c.var2.assembly.name

#raw_input('hit enter to start planning.')
#ExecutePlan(env,youbots,min_regrasp_assignment,min_regrasp_constraints,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose,planners,robot_base_homes)



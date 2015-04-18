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
from wingdemoenv2 import *

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
#op_pose = np.eye(4)
# frameenv op_pose
#op_pose[2,3] = 0.35 
# wingdemoenv op_pose
op_pose = np.array([[ 0.7009, -0.7133,  0.    , -1.2478],
                    [ 0.7133,  0.7009,  0.    ,  0.3399],
                    [ 0.    ,  0.    ,  1.    ,  0.13  ],
                    [ 0.    ,  0.    ,  0.    ,  1.    ]])

variables,asm_op_variables = anytime_planner.ComputeVariables(assembly_operations)
values = anytime_planner.ComputeAllValues(env,yik,variables,op_pose)

youbotenv.ReplaceWithHiResYoubot('drc1')
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
                v.parent_operation.Visualize(op_pose)
                break
        time.sleep(3.0)
        for config in values[c]:
            config.Visualize(env,'drc1_hires')
            time.sleep(0.05)
    c = raw_input('which var?')


##raw_input('hit enter to start planning')
#time_budget_minutes = 2.0
#min_regrasp_assignment,min_regrasp_constraints,variables,asm_op_variables = anytime_planner.Plan(env,yik,op_pose,assembly_operations,all_robot_names,time_budget_minutes)
#if min_regrasp_assignment is None:
#    raise Exception('Assembly grasp planning failed')
#
##raw_input('hit enter to visualize')
#
#def VisualizePlan(env,plan,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose):
#    away = np.eye(4)
#    away[0,3]=1000.0
#    orig_part_poses = {}
#    for o in all_object_names:
#        orig_part_poses[o] = env.GetKinBody(o).GetTransform()
#    for assembly_operation in assembly_operations:
#        # Move robots and parts away.
#        for r in all_robot_names:
#            env.GetRobot(r).SetTransform(away)
#        for o in all_object_names:
#            env.GetKinBody(o).SetTransform(away)
#        rem_robots = copy.copy(all_robot_names)
#        op_vars = asm_op_variables[assembly_operation.name]
#        #IPython.embed()
#        for v in chain.from_iterable(op_vars.values()):
#            plan[str(v)].Visualize(env,rem_robots[-1])
#            del rem_robots[-1]
#        this_op_pose = op_pose
#        if type(assembly_operation) is Part:
#            this_op_pose = orig_part_poses[assembly_operation.name] # Use current transform of part in openrave world.
#        assembly_operation.Visualize(this_op_pose)
#        raw_input('Hit enter for next step.')
#
##if not (min_regrasp_assignment is None):
##    VisualizePlan(env,min_regrasp_assignment,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose)
#
#def ChooseClosest(pose,robot_names,youbots):
#    min_dist_sqr = sys.float_info.max
#    min_dist_robot_name = ''
#    for n in robot_names:
#        diff = youbots[n].GetTransform()[:3,3]-pose[:3,3]
#        dist_sqr = diff.x**2 + diff.y**2
#        if dist_sqr <= min_dist_sqr:
#            min_dist_robot_name = n
#            min_dist_sqr = dist_sqr
#    return min_dist_robot_name
#
#def IsTransferTarget(v,constraints):
#    for c in constraints:
#        if type(c) is CollisionConstraints.BusrideConstraint:
#            if v == c.var1:
#                #print 'Transfer target! ',v.assembly.name
#                return True
#    #print 'NOT Transfer target! ',v.assembly.name
#    return False
#
#def IsTransferConstraint(source,target,constraints):
#    for c in constraints:
#        if type(c) is CollisionConstraints.BusrideConstraint:
#            if target == c.var1 and source == c.var2:
#                #print 'Transfer constraint! s:, ',source.assembly.name,', t: ',target.assembly.name
#                return True
#    #print 'NOT! Transfer constraint! s:, ',source.assembly.name,', t: ',target.assembly.name
#    return False
#
#def ChooseRobot(robots):
#    return random.choice(robots)
#
#def AssignRobotsToOperations(variables,constraints,asm_op_variables,all_robot_names,assembly_operations):
#    # grasp, transfer, regrasp, transfer_release, regrasp_release
#    robot_ops_for_vars = {}
#    for v in variables:
#        if type(v.parent_operation) is Part:
#            robot_ops_for_vars[v] = 'grasp'
#        elif IsTransferTarget(v,constraints):
#            robot_ops_for_vars[v] = 'transfer_release'
#        else:
#            robot_ops_for_vars[v] = 'regrasp_release'
#    var_connections = {} # mapping from one var to the prev var sharing the same robot
#    for v in variables:
#        #if type(v.parent_operation) is Part:
#        if robot_ops_for_vars[v] == 'grasp':
#            continue
#        elif robot_ops_for_vars[v] == 'regrasp' or robot_ops_for_vars[v] == 'regrasp_release':
#            if not (type(v.assembly) is Part):
#                transferable_subasm_list = [x for x in v.assembly.assembly_op_list if (not ((type(x) is Part) and (not x.is_transferable)))]
#            else:
#                transferable_subasm_list = copy.copy(v.assembly.assembly_op_list)
#            subasm = random.choice(transferable_subasm_list)
#            choice_var = asm_op_variables[v.assembly.name][subasm.name][0] # Just takes the first grasp if there are multiple. Should not matter for regrasp.
#        else: # 'transfer' or 'transfer_release'
#            subasm_variables = [v2 for v2 in chain.from_iterable(asm_op_variables[v.assembly.name].values()) ]
#            for subasm_v in subasm_variables:
#                if IsTransferConstraint(source=subasm_v,target=v,constraints=constraints):
#                    choice_var = subasm_v
#                    break
#        if robot_ops_for_vars[choice_var] == 'transfer_release':
#            robot_ops_for_vars[choice_var] = 'transfer'
#        elif robot_ops_for_vars[choice_var] == 'regrasp_release': 
#            robot_ops_for_vars[choice_var] = 'regrasp'
#        var_connections[v] = choice_var
#    robots_for_vars = {}
#    free_robots = copy.copy(all_robot_names)
#    for op in assembly_operations:
#        op_variables = [v for v in chain.from_iterable(asm_op_variables[op.name].values()) ]
#        for v in op_variables:
#            if robot_ops_for_vars[v] == 'grasp':
#                rob = ChooseRobot(free_robots)
#                robots_for_vars[v] = rob
#                free_robots.remove(rob)
#            else:
#                robots_for_vars[v] = robots_for_vars[var_connections[v]]
#                if (robot_ops_for_vars[v] == 'regrasp_release') or (robot_ops_for_vars[v] == 'transfer_release'):
#                    free_robots.append(robots_for_vars[v])
#    return robots_for_vars,robot_ops_for_vars
#
#def EnableGripperLinks(env,robot_name,enable):
#    with env:
#        rob = env.GetRobot(robot_name)
#        rob.GetLink('leftgripper').Enable(enable)
#        rob.GetLink('rightgripper').Enable(enable)
#        rob.GetLink('link5').Enable(enable)
#
#def MoveBaseTo(robot,goal,planner):
#    xyyaw = np.array([goal[0,3],goal[1,3],np.arctan2(goal[1,0],goal[0,0])])
#    current = robot.GetTransform()
#    currentxyyaw = np.array([current[0,3],current[1,3],np.arctan2(current[1,0],current[0,0])])
#    if np.linalg.norm(currentxyyaw-xyyaw) < 0.001:
#        print robot.GetName(),' already at goal. Not moving.'
#        return 
#    with env:
#        robot.SetActiveDOFs([],orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
#        traj = planner.MoveActiveJoints(goal=xyyaw,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True)
#        robot.GetController().SetPath(traj)
#    while not robot.GetController().IsDone():
#        time.sleep(0.01)
#
#def MoveArmTo(robot,goal,planner):
#    with env:
#        robot.SetActiveDOFs(range(5))
#        planner.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2)
#    while not robot.GetController().IsDone():
#        time.sleep(0.01)
#
#def GrabAssembly(env,robot_name,assembly,grab=True):
#    robot = env.GetRobot(robot_name)
#    for part in assembly.GetPartList():
#        if grab:
#            robot.Grab(part.body)
#        else:
#            robot.Release(part.body)
#
#def Regrasp(env,robot,assembly,new_pose_in_gripper):
#    GrabAssembly(env,robot.GetName(),assembly,False)
#    gripper_pose = robot.GetManipulators()[0].GetEndEffectorTransform()
#    new_assembly_pose = np.dot(gripper_pose,new_pose_in_gripper)
#    assembly.SetTransform(new_assembly_pose)
#    GrabAssembly(env,robot.GetName(),assembly,True)
#
#def Execute(env,youbots,robot_name,op_name,assembly,config,planners,robot_base_homes):
#    EnableGripperLinks(env,robot_name,False)
#    # Move home
#    # MoveBaseTo(youbots[robot_name],robot_base_homes[robot_name],planners[robot_name]) 
#    if op_name == 'regrasp' or op_name == 'regrasp_release':
#        print 'REGRASP!'
#        MoveArmTo(youbots[robot_name],np.array([0.,0.,0.,0.,0.]),planners[robot_name]) 
#        new_ee_pose_in_assembly = np.dot(assembly.GetPartPoseInAssembly(config.part_grasp.part_name),config.part_grasp.grasp)
#        regrasped_asm_in_gripper = np.linalg.inv(new_ee_pose_in_assembly)
#        Regrasp(env,youbots[robot_name],assembly,regrasped_asm_in_gripper) 
#    # Move to config
#    #Go 5 cm away from target
#    with env:
#        with youbots[robot_name]:
#            old_arm = copy.deepcopy( config.arm_config )
#                
#            dofs = youbots[robot_name].GetDOFValues()
#            dofs[0:5] = old_arm
#            youbots[robot_name].SetDOFValues(dofs)
#            transform = youbots[robot_name].GetManipulators()[0].GetEndEffectorTransform()
#            vec = np.zeros((3, 1))
#            vec[2] = BACKUP_AMT
#            diff = np.dot(transform[:-1, :-1], vec)
#            transform2 = copy.deepcopy(transform)
#            transform2[0, 3] -= diff[0]
#            transform2[1, 3] -= diff[1]
#            transform2[2, 3] -= diff[2]
#                
#    blah = orpy.misc.DrawAxes(env,transform2,0.25,2.0)   
#    sol = yik.FindIKSolutions(youbots[robot_name], transform2)
#    if len(sol) > 0:    
#        target_arm = sol[0]
#        #plan[str(v)].arm_config = target_arm
#        
#        MoveArmTo(youbots[robot_name],target_arm,planners[robot_name])
#        MoveBaseTo(youbots[robot_name],config.base_config,planners[robot_name])
#        MoveArmTo(youbots[robot_name],config.arm_config,planners[robot_name]) 
#    else:
#        MoveArmTo(youbots[robot_name],config.arm_config,planners[robot_name]) 
#        MoveBaseTo(youbots[robot_name],config.base_config,planners[robot_name])
#         
#    
#    EnableGripperLinks(env,robot_name,True)
#    if op_name == 'grasp':
#        GrabAssembly(env,robot_name,assembly,True)
#
#def PrioritizeNontrivialVars(op_vars):
#    ordered_op_vars = []
#    parts = []
#    for v in op_vars:
#        if type(v.assembly) is Part:
#            parts.append(v)
#        else:
#            ordered_op_vars.append(v)
#    # HACK TO MOVE FASTENERS LAST
#    parts2 = []
#    for v in parts:
#        if 'fastener' in v.assembly.name:
#            parts2.append(v)
#        else:
#            parts2.insert(0,v)
#    parts = parts2
#    # HACK TO MOVE FASTENERS LAST - ENDS HERE
#    ordered_op_vars.extend(parts)
#    return ordered_op_vars
#
#def ExecutePlan(env,youbots,plan,constraints,assembly_operations,variables,asm_op_variables,all_robot_names,all_object_names,op_pose,planners,robot_base_homes):
#    robots_for_vars,robot_ops_for_vars = AssignRobotsToOperations(variables,constraints,asm_op_variables,all_robot_names,assembly_operations)
#    asm_to_robots = {}
#    for op in assembly_operations:
#        # order variables such that nontrivial ops get priority 
#        op_vars = [v for v in chain.from_iterable(asm_op_variables[op.name].values()) ]
#        ordered_op_vars = PrioritizeNontrivialVars(op_vars)
#        for v in ordered_op_vars: # XXX this assumes one robot per assembly.
#            #print str(v),' config: ',plan[str(v)].base_config
#            
#            #Back up:
#            robot = youbots[ robots_for_vars[v] ]
#            with env:
#                with robot:
#                    transform = robot.GetManipulators()[0].GetEndEffectorTransform()
#                    vec = np.zeros((3, 1))
#                    vec[2] = BACKUP_AMT
#                    diff = np.dot(transform[:-1, :-1], vec)
#                    transform2 = copy.deepcopy(transform)
#                    transform2[0, 3] -= diff[0]
#                    transform2[1, 3] -= diff[1]
#                    transform2[2, 3] -= diff[2]
#                    
#            blah = orpy.misc.DrawAxes(env,transform2,0.25,2.0)   
#            sol = yik.FindIKSolutions(robot, transform2)
#            if len(sol) > 0:    
#                target_arm = sol[0]
#                MoveArmTo(robot,target_arm,planners[robots_for_vars[v]]) 
#            
#            Execute(env,youbots,robots_for_vars[v],robot_ops_for_vars[v],v.assembly,plan[str(v)],planners,robot_base_homes)
#
#
#        for v in ordered_op_vars:
#            op_name = robot_ops_for_vars[v]
#            r = robots_for_vars[v]
#            if op_name == 'transfer_release' or op_name == 'regrasp_release':
#                # Release assembly
#                GrabAssembly(env,r,v.assembly,False)
#                EnableGripperLinks(env,r,False)
#                MoveBaseTo(youbots[r],robot_base_homes[r],planners[r]) 
#                EnableGripperLinks(env,r,True)
#                #raw_input('Hit enter for next execution.')
#            elif op_name == 'transfer' or op_name == 'regrasp':
#                GrabAssembly(env,r,v.assembly,False) # Release the subassembly 
#                GrabAssembly(env,r,v.parent_operation,True) # Grab the full thing.
#
#IPython.embed()
#
#for c in min_regrasp_constraints:
#    if type(c) is CollisionConstraints.BusrideConstraint:
#        print 'Busride between: ',c.var1.assembly.name,' and ',c.var2.assembly.name
#
##IPython.embed()
#raw_input('hit enter to execute.')
#ExecutePlan(env,youbots,min_regrasp_assignment,min_regrasp_constraints,assembly_operations,variables,asm_op_variables,all_robot_names,all_object_names,op_pose,planners,robot_base_homes)
#
##if not (min_regrasp_assignment is None):
##    VisualizePlan(env,min_regrasp_assignment,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose)
#

IPython.embed()

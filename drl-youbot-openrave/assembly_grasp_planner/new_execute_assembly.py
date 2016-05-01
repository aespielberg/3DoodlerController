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
import datetime

import openravepy as orpy
import youbotpy
from youbotpy import youbotik as yik
import GraspGenerator
import AnytimePlanner
import CollisionConstraints
import copy
import tfplugin

#from randomarrayenv import *
#from randomarrayenv import *
#from simpleenv import *
#from simpleenv_velcro import *
from table import *

#from chairenv3 import *

#from pictureenv import *
#from wingdemoenv import *

BACKUP_AMT = 0.03
LIFT_AMT = 0.3
MOVE_AMT = 0.1

start_var_name = None
#start_var_name = 'seat__seat__0'

use_pickled_plan = True
#pickled_plan_file_name = 'corrected_velcro.p'
pickled_plan_file_name = 'tableplan.p'
sim = False
use_vicon_for_start_poses = False
use_boots = False

youbotenv = youbotpy.YoubotEnv(sim=sim,viewer=True,env_xml=envfile, \
                               youbot_names=all_robot_names, registered_objects=[])

env = youbotenv.env
youbots = youbotenv.youbots
if sim:
    for name in youbots:
        youbots[name].SetTransform(robot_start_poses[name])
        youbotenv.MoveGripper(name,0.01,0.01) # open grippers

yik.init(youbots[youbots.keys()[0]]) #   does not matter which robot
                                     # we use to initialize since 
                                     # all youbots have same kinematics.
grasp_generator = GraspGenerator.GraspGenerator(env)
pickle_plan = []

if use_vicon_for_start_poses:
    youbotenv.GetRealStartPoses(youbot_names=[],objects=[o for o in all_object_names if not ('fastener' in o or 'leftside' in o or 'cseat' in o)])

if (use_vicon_for_start_poses or not sim) and use_boots:
    youbotenv.tfplugin.RegisterBody(env.GetKinBody('boot_rightside'),'rightside')
    youbotenv.tfplugin.RegisterBody(env.GetKinBody('boot_back'),'back')
    youbotenv.tfplugin.RegisterBody(env.GetKinBody('bootextension_rightside'),'rightside')
    youbotenv.tfplugin.RegisterBody(env.GetKinBody('bootextension_back'),'back')
    #youbotenv.tfplugin.RegisterBody(env.GetKinBody('boot_leftside'),'leftside')
    #youbotenv.tfplugin.RegisterBody(env.GetKinBody('boot_seat'),'seat')
    #youbotenv.tfplugin.RegisterBody(env.GetKinBody('bootextension_leftside'),'leftside')
    #youbotenv.tfplugin.RegisterBody(env.GetKinBody('bootextension_seat'),'seat')
    #youbotenv.tfplugin.RegisterBody(env.GetKinBody('boot_leftside'),'leftside')
    #youbotenv.tfplugin.RegisterBody(env.GetKinBody('boot_seat'),'seat')

time.sleep(3.0)

#if not sim:
    #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('back'))
    #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('rightside'))
    #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('seat'))
    #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('leftside'))

if (use_vicon_for_start_poses or not sim) and use_boots:
        youbotenv.tfplugin.UnregisterBody(env.GetKinBody('boot_rightside'))
        youbotenv.tfplugin.UnregisterBody(env.GetKinBody('boot_back'))
        youbotenv.tfplugin.UnregisterBody(env.GetKinBody('bootextension_rightside'))
        youbotenv.tfplugin.UnregisterBody(env.GetKinBody('bootextension_back'))
        #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('boot_leftside'))
        #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('boot_seat'))
        #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('bootextension_leftside'))
        #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('bootextension_seat'))
        #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('boot_leftside'))
        #youbotenv.tfplugin.UnregisterBody(env.GetKinBody('boot_seat'))

#env.GetKinBody('boot_back').SetTransform(env.GetKinBody('back').GetTransform())
#env.GetKinBody('bootextension_back').SetTransform(env.GetKinBody('back').GetTransform())
#env.GetKinBody('boot_rightside').SetTransform(env.GetKinBody('rightside').GetTransform())
#env.GetKinBody('bootextension_rightside').SetTransform(env.GetKinBody('rightside').GetTransform())


for part in all_parts:
    part.body = env.GetKinBody(part.name)
    part.SetGrasps(grasp_generator.GeneratePartGrasps(part))

robot_base_homes = {}
for r in all_robot_names:
    robot_base_homes[r] = youbots[r].GetTransform()

planners = {}
for r in all_robot_names:
    planners[r] = orpy.interfaces.BaseManipulation(youbots[r],plannername='BiRRT')



if start_var_name is None:
    for r in all_robot_names:
        # open hands
        youbotenv.MoveArm(r,np.array([ 0.,0.,0.,0.,0.]))
        youbotenv.MoveGripper(r, 0.01, 0.01)

anytime_planner = AnytimePlanner.AnytimePlanner()

#raw_input('hit enter to visualize')

def GetDistance(pose1,pose2):
    # FIXME For now ignoring rotational distance.
    return np.linalg.norm(pose1[:3,3]-pose2[:3,3])

def FindNearbyIKSolution(robot,ee_pose):
    sol = yik.FindIKSolutions(robot, ee_pose)
    if len(sol) > 0:    
        current_arm = robot.GetDOFValues()[0:5]
        target_arm = GetClosestArm(current_arm, sol)
        # Check if the change is too big per joint (and translation) and warn. 
        for i in range(5):
            if abs(current_arm[i]-target_arm[i]) > np.pi/36.0:
                print 'FindNearbyIK: Change in joint ',i,' to big: ',abs(current_arm[i]-target_arm[i])
                return None
        print 'FindNearbyIK: Found nearby ik'
        return target_arm
    else:
        print 'FindNearbyIK: Failed to find any ik solution.'
        return None

def MoveEndEffectorTo(robot,hand,target_pose,planner,threshold=0.003):
    while True:
        offset = GetDistance(hand.GetTransform(),target_pose)
        if offset < threshold:
            print 'End-effector is at requested pose.'
            break
        print 'End-effector off by ',offset
        robot_ee = robot.GetManipulators()[0].GetEndEffectorTransform()
        target_robot_ee = robot_ee + (target_pose - hand.GetTransform()[:3,3]) # FIXME for now only translational.
        soln = FindNearbyIKSolution(robot,target_robot_ee)
        if soln is None:
            print 'Failed to move to ee pose. Giving up.'
            return False
        else:
            print 'Moving to ee pose.'
            MoveArmTo(robot,soln,planner)
    return True
         
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
                #print 'Transfer target! ',v.assembly.name
                return True
    #print 'NOT Transfer target! ',v.assembly.name
    return False

def IsTransferConstraint(source,target,constraints):
    for c in constraints:
        if type(c) is CollisionConstraints.BusrideConstraint:
            if target == c.var1 and source == c.var2:
                #print 'Transfer constraint! s:, ',source.assembly.name,', t: ',target.assembly.name
                return True
    #print 'NOT! Transfer constraint! s:, ',source.assembly.name,', t: ',target.assembly.name
    return False

def ChooseRobot(robots):
    return random.choice(robots)

def AssignRobotsToOperations(variables,constraints,asm_op_variables,all_robot_names,assembly_operations):
    # grasp, transfer, regrasp, transfer_release, regrasp_release
    robot_ops_for_vars = {}
    for v in variables:
        if type(v.parent_operation) is Part:
            robot_ops_for_vars[str(v)] = 'grasp'
        elif IsTransferTarget(v,constraints):
            robot_ops_for_vars[str(v)] = 'transfer_release'
        else:
            robot_ops_for_vars[str(v)] = 'regrasp_release'
    var_connections = {} # mapping from one var to the prev var sharing the same robot
    for v in variables:
        #if type(v.parent_operation) is Part:
        if robot_ops_for_vars[str(v)] == 'grasp':
            continue
        elif robot_ops_for_vars[str(v)] == 'regrasp' or robot_ops_for_vars[str(v)] == 'regrasp_release':
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
        if robot_ops_for_vars[str(choice_var)] == 'transfer_release':
            robot_ops_for_vars[str(choice_var)] = 'transfer'
        elif robot_ops_for_vars[str(choice_var)] == 'regrasp_release': 
            robot_ops_for_vars[str(choice_var)] = 'regrasp'
        var_connections[v] = choice_var
    robots_for_vars = {}
    free_robots = copy.copy(all_robot_names)
    for op in assembly_operations:
        op_variables = [v for v in chain.from_iterable(asm_op_variables[op.name].values()) ]
        for v in op_variables:
            if robot_ops_for_vars[str(v)] == 'grasp':
                rob = ChooseRobot(free_robots)
                robots_for_vars[str(v)] = rob
                free_robots.remove(rob)
            else:
                robots_for_vars[str(v)] = robots_for_vars[str(var_connections[v])]
                if (robot_ops_for_vars[str(v)] == 'regrasp_release') or (robot_ops_for_vars[str(v)] == 'transfer_release'):
                    free_robots.append(robots_for_vars[str(v)])
    return robots_for_vars,robot_ops_for_vars

def EnableGripperLinks(env,robot_name,enable):
    with env:
        rob = env.GetRobot(robot_name)
        rob.GetLink('leftgripper').Enable(enable)
        rob.GetLink('rightgripper').Enable(enable)
        rob.GetLink('link5').Enable(enable)
        
def EnableGrabbedObjects(env,robot_name,enable):
    with env:
        rob = env.GetRobot(robot_name)
        for o in rob.GetGrabbed():
            o.Enable(enable)
 
def MoveBack(robot,planner):
    try:
        EnableGripperLinks(robot.GetEnv(),robot.GetName(),False)
        goal = robot.GetTransform()
        ee = robot.GetManipulators()[0].GetEndEffectorTransform()
        goal[:2,3] -= ee[:2,2]*0.10 # Move back 10 cm (before projection)
        xyyaw = np.array([goal[0,3],goal[1,3],np.arctan2(goal[1,0],goal[0,0])])
        with env:
            robot.SetActiveDOFs([],orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
            traj = planner.MoveActiveJoints(goal=xyyaw,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True, jitter=-0.01)
            robot.GetController().SetPath(traj)
        while not robot.GetController().IsDone():
            time.sleep(0.01)
    except Exception, e:
        print 'in moveback got exception.'
        print str(e)
        c = raw_input('IPython shell?(y/n)')
        if c == 'y':
            IPython.embed()
        raise e
    finally:
        EnableGripperLinks(robot.GetEnv(),robot.GetName(),True)


def MoveBaseTo(robot,goal,planner,skip_waypoints=False):
    try:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Verbose)
        robot.GetEnv().GetKinBody('floor').Enable(False)
        xyyaw = np.array([goal[0,3],goal[1,3],np.arctan2(goal[1,0],goal[0,0])])
        current = robot.GetTransform()
        currentxyyaw = np.array([current[0,3],current[1,3],np.arctan2(current[1,0],current[0,0])])
        if np.linalg.norm(currentxyyaw-xyyaw) < 0.0001:
            print robot.GetName(),' already at goal. Not moving.'
            return 
        with env:
            robot.SetActiveDOFs([],orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
            traj = planner.MoveActiveJoints(goal=xyyaw,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True, jitter=-0.01)
            if skip_waypoints:
                traj.Remove(1,traj.GetNumWaypoints()-1)
            robot.GetController().SetPath(traj)
        while not robot.GetController().IsDone():
            time.sleep(0.01)
    except Exception, e:
        print str(e)
        if sim:
            print 'Snapping to goal.'
            robot.SetTransform(goal)
        else:
            MoveBack(robot,planner)
            #c = raw_input('IPython shell?(y/n)')
            #if c == 'y':
            #    IPython.embed()
            #raise e
    finally:
        robot.GetEnv().GetKinBody('floor').Enable(True)
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Info)

def getArmNorm(traj, robot):
    traj_start = traj.GetWaypoint(0)
    traj_end = traj.GetWaypoint( traj.GetNumWaypoints() - 1 ) #get last waypoint
    
    first_config = traj.GetConfigurationSpecification().ExtractJointValues(traj_start, robot, robot.GetActiveDOFIndices())
    last_config = traj.GetConfigurationSpecification().ExtractJointValues(traj_end, robot, robot.GetActiveDOFIndices())
    
    return np.linalg.norm(first_config - last_config, np.inf) #supnorm

def MoveArmTo(robot,goal,planner):
    try:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Verbose)
        with env:
            robot.SetActiveDOFs(range(5))
            print 'BEFORE moveactivejoints'
            traj = planner.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True, jitter=-0.01)
            print 'AFTER moveactivejoints'
            robot.GetController().SetPath(traj)
        while not robot.GetController().IsDone():
            time.sleep(0.01)
    finally:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Info)

def GrabAssembly(env,robot_name,assembly,grab=True,move_gripper=True):
    for part in assembly.GetPartList():
        if grab:
            raw_input('hit enter')
            youbotenv.Grab(robot_name, part.body,move_gripper)
        else:
            youbotenv.Release(robot_name, part.body,move_gripper)

def Regrasp(env,robot,assembly,new_pose_in_gripper):
    GrabAssembly(env,robot.GetName(),assembly,False)
    gripper_pose = robot.GetManipulators()[0].GetEndEffectorTransform()
    new_assembly_pose = np.dot(gripper_pose,new_pose_in_gripper)
    assembly.SetTransform(new_assembly_pose)
    GrabAssembly(env,robot.GetName(),assembly,True)
    
def GetClosestArm(cur_arm, sol):
    best_dist = sys.maxint
    ret_i = 0
    for (i, v) in enumerate(sol):
        dist = np.linalg.norm(v - cur_arm, np.inf )
        if dist < best_dist:
            best_dist = dist
            ret_i = i
    print 'getting closest arm'
    #IPython.embed()
    return sol[ret_i]
    
def GetClosestArmIdx(cur_arm, sol):
    best_dist = sys.maxint
    ret_i = 0
    for (i, v) in enumerate(sol):
        dist = np.linalg.norm(v - cur_arm, np.inf )
        if dist < best_dist:
            best_dist = dist
            ret_i = i
    print 'getting closest arm'
    #IPython.embed()
    return ret_i
    
def extract_plan(robot_name, op_name, config):
    step_tup = [None, None, None, None]
    if op_name == 'transfer_release' or op_name == 'regrasp_release':
        step_tup[0] = 'release'
    elif op_name == 'regrasp' or op_name == 'transfer':
        step_tup[0] = 'regrasp'
    elif op_name == 'grasp':
        step_tup[0] = 'grasp'
    step_tup[1] = robot_name
    step_tup[2] = config.arm_config
    step_tup[3] = config.base_config
    pickle_plan.append(step_tup)
        
        
def Execute(env,youbots,robot_name,op_name,assembly,config,planners,robot_base_homes):
    ##EnableGripperLinks(env,robot_name,False)
    #extract_plan(robot_name, op_name, config)
    # Move home
    # MoveBaseTo(youbots[robot_name],robot_base_homes[robot_name],planners[robot_name]) 
    if op_name == 'regrasp' or op_name == 'regrasp_release':
        print 'REGRASP!'
        MoveArmTo(youbots[robot_name],np.array([0.,0.,0.,0.,0.]),planners[robot_name]) 
        new_ee_pose_in_assembly = np.dot(assembly.GetPartPoseInAssembly(config.part_grasp.part_name),config.part_grasp.grasp)
        regrasped_asm_in_gripper = np.linalg.inv(new_ee_pose_in_assembly)
        Regrasp(env,youbots[robot_name],assembly,regrasped_asm_in_gripper) 
    # Move to config
    #Go 5 cm away from target
    #IPython.embed()
    if not (op_name.startswith('fastener')):
        #raw_input('Move to backed up pose')
        with env:
            with youbots[robot_name]:
                #h = []
                old_arm = copy.deepcopy( config.arm_config )
                dofs = youbots[robot_name].GetDOFValues()
                dofs[0:5] = old_arm
                youbots[robot_name].SetDOFValues(dofs)
                youbots[robot_name].SetTransform(config.base_config)
                EnableGrabbedObjects(env,robot_name,False)
                EnableGripperLinks(env,robot_name,False)
                if op_name == 'transfer' or op_name == 'transfer_release':
                    backed_up = BackupHandConfigInZ(env,robot_name,n_steps=20)
                else:
                    backed_up = BackupHandConfig(env,robot_name,n_steps=20)
                EnableGripperLinks(env,robot_name,True)
                EnableGrabbedObjects(env,robot_name,True)
        if not (backed_up is None) and not (len(backed_up) == 0):
            #IPython.embed()
            #floor_pose = env.GetKinBody('floor').GetTransform()
            #floor_pose[2,3] += 0.015
            #env.GetKinBody('floor').SetTransform(floor_pose)
            MoveArmTo(youbots[robot_name],backed_up[0],planners[robot_name])
            #floor_pose[2,3] -= 0.015
            #env.GetKinBody('floor').SetTransform(floor_pose)
            #raw_input('Move to actual pose')
            #if 'rightside' == config.part_grasp.part_name:
            #    EnableGripperLinks(env,robot_name,False)
            MoveBaseTo(youbots[robot_name],config.base_config,planners[robot_name])
            #time.sleep(0.3)
            #MoveBaseTo(youbots[robot_name],config.base_config,planners[robot_name])
            #time.sleep(0.3)
            #MoveBaseTo(youbots[robot_name],config.base_config,planners[robot_name])
            adjust = False
            if adjust: # TODO not working yet.
                raw_input('Adjust end-effector pose.')
                target_pose = np.dot(env.GetTransform(config.part_grasp.part_name),config.part_grasp.grasp)
                target_backed_up_pose = target_pose.copy()
                target_backed_up_pose[:3,3] = target_backed_up_pose[:3,3] - (BACKUP_AMT*target_backed_up_pose[:3,2])
                MoveEndEffectorTo(youbots[robot_name],youbotenv.youbot_hands[robot_name],target_backed_up_pose,planners[robot_name])
            #if 'rightside' == config.part_grasp.part_name:
            #    EnableGripperLinks(env,robot_name,True)
            #raw_input('jerk forward 5cm')
            EnableGripperLinks(env,robot_name,False)
            EnableGrabbedObjects(env,robot_name,False)
            env.GetKinBody(config.part_grasp.part_name).Enable(False)
            if adjust:
                forward_configs = MoveForwardHandConfig(env,robot_name,n_steps=20)
                backed_up = forward_configs
            print 'BEFORE for b in backedup'
            for b in backed_up[1:]:
                print 'IN for b in backedup'
                MoveArmTo(youbots[robot_name],b,planners[robot_name]) 
            if not adjust: # FIXME do we need this?
                MoveArmTo(youbots[robot_name],config.arm_config,planners[robot_name]) 
            env.GetKinBody(config.part_grasp.part_name).Enable(True)
            EnableGrabbedObjects(env,robot_name,True)
            EnableGripperLinks(env,robot_name,True)
            #IPython.embed()
        else:
            #raise Exception('Cannot go to backed up pose.')
            raw_input('Move to totally final pose')
            MoveArmTo(youbots[robot_name],config.arm_config,planners[robot_name])
            env.GetKinBody(config.part_grasp.part_name).Enable(False)
            MoveBaseTo(youbots[robot_name],config.base_config,planners[robot_name])
            env.GetKinBody(config.part_grasp.part_name).Enable(True)
    else:
        raw_input('Move to totally final pose')
        MoveArmTo(youbots[robot_name],config.arm_config,planners[robot_name])
        env.GetKinBody(config.part_grasp.part_name).Enable(False)
        MoveBaseTo(youbots[robot_name],config.base_config,planners[robot_name])
        env.GetKinBody(config.part_grasp.part_name).Enable(True)
    
    ##EnableGripperLinks(env,robot_name,True)
    if op_name == 'grasp':
        if 'fastener' in config.part_grasp.part_name:
            raw_input('Ready to grasp')
            time.sleep(3.0)
        #raw_input('hit enter')
        GrabAssembly(env,robot_name,assembly,True)
        
        if not ('fastener' in config.part_grasp.part_name):
            xspace = np.linspace(0.,0.05,20)
            yspace = np.linspace(0.,0.05,20)
            xyspace = list(np.transpose([np.tile(xspace, len(yspace)), np.repeat(yspace, len(xspace))]))
            for xy in xyspace:
                lift_amt = 0.02
                transform = youbots[robot_name].GetManipulators()[0].GetEndEffectorTransform()
                transform[2, 3] += lift_amt
                transform[0, 3] += xy[0]
                transform[1, 3] += xy[1]
                sol = yik.FindIKSolutions(youbots[robot_name], transform)
                if len(sol) > 0:    
                    target_arm = GetClosestArm(youbots[robot_name].GetDOFValues()[0:5], sol)
                    MoveArmTo(youbots[robot_name],target_arm,planners[robot_name])
                    break
                else:
                    print 'Lift attempt failed'

        #MoveBaseTo(youbots[robot_name], base[idx],planners[robot_name])
                

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

def BackupHandConfigInZ(env,r,n_steps=1):
    robot = youbots[ r ]
    solns = []
    rn = range(n_steps)
    rn.reverse()
    for step in rn:
        with env:
            with robot:
                transform = robot.GetManipulators()[0].GetEndEffectorTransform()
                z = BACKUP_AMT*(float(step+1)/float(n_steps))
                transform2 = copy.deepcopy(transform)
                transform2[2, 3] += z
                sol = yik.FindIKSolutions(robot, transform2)
        if len(sol) > 0:    
            print 'Found solution for back up in z ',z
            target_arm = GetClosestArm(robot.GetDOFValues()[0:5], sol)
            if n_steps == 1:
                return target_arm
            else:
                solns.append(target_arm)
        else:
            if n_steps == 1:
                return None
            else:
                continue
    return solns

def BackupHandConfig(env,r,n_steps=1):
    robot = youbots[ r ]
    solns = []
    rn = range(n_steps)
    rn.reverse()
    for step in rn:
        with env:
            with robot:
                transform = robot.GetManipulators()[0].GetEndEffectorTransform()
                vec = np.zeros((3, 1))
                vec[2] = BACKUP_AMT*(float(step+1)/float(n_steps))
                diff = np.dot(transform[:-1, :-1], vec)
                transform2 = copy.deepcopy(transform)
                transform2[0, 3] -= diff[0]
                transform2[1, 3] -= diff[1]
                transform2[2, 3] -= diff[2]
                #blah = orpy.misc.DrawAxes(env,transform2,0.25,2.0)   
                sol = yik.FindIKSolutions(robot, transform2)
                #IPython.embed() 
        if len(sol) > 0:    
            print 'Found solution for back up ',vec[2]
            target_arm = GetClosestArm(robot.GetDOFValues()[0:5], sol)
            if n_steps == 1:
                return target_arm
            else:
                solns.append(target_arm)
        else:
            if n_steps == 1:
                return None
            else:
                continue
    return solns

def MoveForwardHandConfig(env,r,n_steps=1):
    robot = youbots[ r ]
    solns = []
    rn = range(n_steps)
    for step in rn:
        with env:
            with robot:
                transform = robot.GetManipulators()[0].GetEndEffectorTransform()
                vec = np.zeros((3, 1))
                vec[2] = BACKUP_AMT*(float(step+1)/float(n_steps))
                diff = np.dot(transform[:-1, :-1], vec)
                transform2 = copy.deepcopy(transform)
                transform2[0, 3] += diff[0]
                transform2[1, 3] += diff[1]
                transform2[2, 3] += diff[2]
                #blah = orpy.misc.DrawAxes(env,transform2,0.25,2.0)   
                sol = yik.FindIKSolutions(robot, transform2)
                #IPython.embed() 
        if len(sol) > 0:    
            print 'Found solution for move forward ',vec[2]
            target_arm = GetClosestArm(robot.GetDOFValues()[0:5], sol)
            if n_steps == 1:
                return target_arm
            else:
                solns.append(target_arm)
        else:
            if n_steps == 1:
                return None
            else:
                continue
    return solns

def OrderVars(op_vars,order):
    ordered_op_vars = []
    for asm_name in order:
        for v in op_vars:
            if v.assembly.name == asm_name:
                ordered_op_vars.append(v)
    return ordered_op_vars

def ExecutePlan(env,youbots,plan,robots_for_vars,robot_ops_for_vars,assembly_operations,variables,asm_op_variables,all_robot_names,all_object_names,op_pose,planners,robot_base_homes):
    global start_var_name
    asm_to_robots = {}
    #raw_input('begin!')
    
    for op in assembly_operations:
        #if start_op_name is not None:
        #    if op.name == start_op_name:
        #        start_op_name = None
        #    else:
        #        continue

        # order variables such that nontrivial ops get priority 
        op_vars = [v for v in chain.from_iterable(asm_op_variables[op.name].values()) ]
        #ordered_op_vars = PrioritizeNontrivialVars(op_vars)
        ordered_op_vars = OrderVars(op_vars,op.order)
        for v in ordered_op_vars: # XXX this assumes one robot per assembly.
            if start_var_name is not None:
                if str(v) == start_var_name:
                    start_var_name = None
                else:
                    continue
            ## XXX Hack: no action during transfer. STARTS HERE
            #op_name = robot_ops_for_vars[str(v)]
            #if op_name == 'transfer':
            #    continue
            ## XXX Hack: no action during transfer. ENDS HERE
            #print str(v),' config: ',plan[str(v)].base_config
            Execute(env,youbots,robots_for_vars[str(v)],robot_ops_for_vars[str(v)],v.assembly,plan[str(v)],planners,robot_base_homes)
        if start_var_name is not None:
            continue

        #if op is assembly_operations[-1]:
        #    ordered_op_vars.reverse()
        for v in ordered_op_vars:
            op_name = robot_ops_for_vars[str(v)]
            r = robots_for_vars[str(v)]
            if op_name == 'transfer_release' or op_name == 'regrasp_release':
                #raw_input('about to release')
                ## XXX drc4 fastener HACK starts 
                #drc4_base = np.array([[ 0.999953  ,  0.00969547,  0.        , -0.78416859],
                #                      [-0.00969547,  0.999953  , -0.        ,  0.01660606],
                #                      [-0.        ,  0.        ,  1.        ,  0.        ],
                #                      [ 0.        ,  0.        ,  0.        ,  1.        ]])
                #MoveArmTo(youbots['drc4'], [0.,  1.3,   0.05, 0.,  -1.5],planners['drc4'])
                #MoveBaseTo(youbots['drc4'],drc4_base,planners['drc4']) 
                #EnableGripperLinks(env,'drc4',False)
                #drc4_base1 = drc4_base.copy()
                #drc4_base2 = drc4_base.copy()
                #drc4_base_backup = drc4_base.copy()
                #drc4_base1[1,3] += 0.025
                #drc4_base2[1,3] += 0.05
                #drc4_base_backup[0,3] -= 0.55
                #IPython.embed()
                #MoveBaseTo(youbots['drc4'],drc4_base1,planners['drc4']) 
                #MoveBaseTo(youbots['drc4'],drc4_base2,planners['drc4']) 
                #youbotenv.MoveGripper('drc4',0.01,0.01) # open grippers
                #MoveBaseTo(youbots['drc4'],drc4_base_backup,planners['drc4']) 
                #EnableGripperLinks(env,'drc4',True)
                ## XXX drc4 fastener HACK ends
                # Release assembly
                if not (r == 'drc2'): # HACK  TODO
                    GrabAssembly(env,r,v.assembly,False)
                    if 'fastener' in v.assembly.name:
                        env.GetKinBody(v.assembly.name).Enable(False)
                        #print 'moving fastener in'
                        ## move fastener into the assembly.
                        #fastener_in = 0.07 * env.GetKinBody(v.assembly.name).GetTransform()[:3,1]
                        #fastener_pose = env.GetKinBody(v.assembly.name).GetTransform()
                        #fastener_pose[:3,3] += fastener_in
                        #env.GetKinBody(v.assembly.name).SetTransform(fastener_pose)
                    EnableGripperLinks(env,r,False)
                    # move hand back 5cm
                    backed_up = BackupHandConfig(env,r,n_steps=2)
                    if not (backed_up is None) and not (len(backed_up) == 0):
                        #raw_input('back up 5cm')
                        MoveArmTo(youbots[r],backed_up[0],planners[r]) 
                        #raw_input('move base back 15 cm')
                        base_backup = youbots[r].GetTransform()
                        base_backup[:2,3] = base_backup[:2,3] - (youbots[r].GetManipulators()[0].GetEndEffectorTransform()[:2,2]*0.45)
                        if not sim:
                            youbots[r].GetController().SendCommand("SetHighPrecision 0")
                        MoveBaseTo(youbots[r],base_backup,planners[r],skip_waypoints=True) 
                        if not sim:
                            youbots[r].GetController().SendCommand("SetHighPrecision 1")
                    EnableGripperLinks(env,r,True)
                    if not sim:
                        youbots[r].GetController().SendCommand("SetHighPrecision 0")
                    #if (r == 'drc1'): # HACK. Remove this later. TODO
                    MoveBaseTo(youbots[r],robot_base_homes[r],planners[r]) 
                    if not sim:
                        youbots[r].GetController().SendCommand("SetHighPrecision 1")
                    if 'fastener' in v.assembly.name:
                        env.GetKinBody(v.assembly.name).Enable(True)
                    #raw_input('Hit enter for next execution.')
            elif op_name == 'transfer' or op_name == 'regrasp':
                GrabAssembly(env,r,v.assembly,False,move_gripper=False) # Release the subassembly 
                GrabAssembly(env,r,v.parent_operation,True,move_gripper=False) # Grab the full thing.


op_pose = np.eye(4)
#op_pose[2,3] = 0.35
op_pose[0,3] = -0.60
op_pose[2,3] = 0.20

#IPython.embed()

#raw_input('hit enter to start planning')
if use_pickled_plan:
    with open(pickled_plan_file_name,'r') as f:
        min_regrasp_assignment = pickle.load(f)
        robots_for_vars = pickle.load(f)
        robot_ops_for_vars = pickle.load(f)
        variables,asm_op_variables = anytime_planner.ComputeVariables(assembly_operations)
else:
    time_budget_minutes = 2.0
    min_regrasp_assignment,min_regrasp_constraints,variables,asm_op_variables = anytime_planner.Plan(env,yik,op_pose,assembly_operations,all_robot_names,time_budget_minutes)
    if min_regrasp_assignment is None:
        raise Exception('Assembly grasp planning failed')
    robots_for_vars,robot_ops_for_vars = AssignRobotsToOperations(variables,min_regrasp_constraints,asm_op_variables,all_robot_names,assembly_operations)

#IPython.embed()

#for c in min_regrasp_constraints:
#    if type(c) is CollisionConstraints.BusrideConstraint:
#        print 'Busride between: ',c.var1.assembly.name,' and ',c.var2.assembly.name

if not use_pickled_plan:
    pickle_filename = str(time.time()) + "save.p" 
    with open( pickle_filename, "wb") as f:
        #pickle.dump( pickle_plan,f)
        pickle.dump(min_regrasp_assignment,f)
        pickle.dump(robots_for_vars,f)
        pickle.dump(robot_ops_for_vars,f)
    print 'pickled to: ', pickle_filename

raw_input('hit enter to execute.')
time.sleep(10.0)
#try:
ExecutePlan(env,youbots,min_regrasp_assignment,robots_for_vars,robot_ops_for_vars,assembly_operations,variables,asm_op_variables,all_robot_names,all_object_names,op_pose,planners,robot_base_homes)
#except Exception, e:
#    print str(e)
#    IPython.embed()
MoveBack(youbots['drc1'],planners['drc1'])
MoveBack(youbots['drc3'],planners['drc3'])


#if not (min_regrasp_assignment is None):
#    VisualizePlan(env,min_regrasp_assignment,assembly_operations,asm_op_variables,all_robot_names,all_object_names,op_pose)


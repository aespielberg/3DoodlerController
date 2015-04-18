#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import openravepy as orpy
import time
from itertools import izip

import IPython
ipshell = IPython.embed

#np.set_printoptions(suppress=True)
#np.set_printoptions(precision=4)

def invert_H(H):
    R = H[0:3,0:3]
    d = H[0:3,3]
    Hinv = np.eye(4)
    Hinv[0:3,0:3] = R.T
    Hinv[0:3,3] = -np.dot(R.T, d)
    return Hinv


def TuckArm(robot):
    tuck_arm_conf = np.array([ 0.    , -0.7547, -0.1766,  0.8878,  0.    ,  0.    ,  0.    ])
    MoveArm(robot,tuck_arm_conf)


def MoveArm(robot, config):
    global simulated
    global animate
    if simulated:
        if animate:
            cur_config = robot.GetDOFValues()
            step = (config-cur_config)/50.0
            for i in range(50):
                time.sleep(0.1)
                robot.SetDOFValues(cur_config+step*i)
        robot.SetDOFValues(config)
    else:
        # TODO send arm commands thru ros
        robot.SetDOFValues(config) # FIXME remove this when robot controller updates form openrave robot.


# Returns tranformations of each part relative to a fixed assembly frame.
def GetAssemblyStructure(assembly_id):
    # FIXME For now returns a fixed structure for the chair back, side and
    # fastener.
    if assembly_id == 'chair_back_and_side':
        chairside = env.GetKinBody('ChairSide')
        chairback = env.GetKinBody('ChairBack')
        fastener = env.GetKinBody('Fastener')

        chairback_pose = np.eye(4)
        chairback_pose[:3,3] = [-0.09, 0.095, 0.15]
        chairside_pose = np.eye(4)
        chairside_pose[:3,3] = [0.0, 0.0, 0.0]
        fastener_pose = np.eye(4)
        fastener_pose[:3,3] = [-0.09, -0.07, 0.28]

        return ([chairback,chairside,fastener],
                [chairback_pose,chairside_pose,fastener_pose])

    else:
        raise Exception('Unknown assembly id: %s'%(assembly_id))


# Returns where the assembly will be achieved in the world.
def GetAssemblyPose():
    # FIXME For now returns a fixed pose.
    pose = np.array([[1.0,  0.0,  0.0,  0.00],
                     [0.0,  1.0,  0.0,  0.00],
                     [0.0,  0.0,  1.0,  0.10],
                     [0.0,  0.0,  0.0,  1.0]])
    return pose


# Parameters:
# robots: the list of robots
# grasp_arm_configs: list of joint angles of each robot during grasp.
# grasps: list of transforms of robot end-effectors relative to parts during
# grasp.
# parts: list of parts to be grasped.
def GraspAssemblyParts(robots, grasp_arm_configs, grasps, parts):
    global simulated
    # FIXME For now parts come to robots magically.
    for (robot,config,grasp,part) in izip(robots,grasp_arm_configs,grasps,parts):
        if simulated:
            robot.SetDOFValues(config)
            manip_in_world = robot.GetManipulators()[0].GetEndEffectorTransform()
            part_in_world = np.dot(manip_in_world,invert_H(grasp))
            part.SetTransform(part_in_world)
            # FIXME Move gripper?
        else:
            # TODO Send arm commands.
            # FIXME Remove the block below when we have controllers updating from real robot and parts.
            robot.SetDOFValues(config) 
            manip_in_world = robot.GetManipulators()[0].GetEndEffectorTransform()
            part_in_world = np.dot(manip_in_world,invert_H(grasp))
            part.SetTransform(part_in_world)
            # TODO Send gripper commands.
        robot.Grab(part)

   
def TransformAssembly(assembly_parts, assembly_parts_relative_poses, assembly_pose):
    for (part,part_relative_pose) in izip(assembly_parts,assembly_parts_relative_poses):
        part.SetTransform(np.dot(assembly_pose,part_relative_pose))


# Returns x,y, and yaw.
def Get2dPose(transform):
    yaw = np.arctan2(transform[1,0],transform[0,0])
    return np.array([transform[0,3],transform[1,3],yaw])


def GetTransform(pose2d):
    pose = np.eye(4)
    pose[0,3] = pose2d[0]  
    pose[1,3] = pose2d[1]  
    pose[0,0] = np.cos(pose2d[2])
    pose[1,0] = np.sin(pose2d[2])
    pose[0,1] = -np.sin(pose2d[2])
    pose[1,1] = np.cos(pose2d[2])
    return pose


def MoveBase(robot,pose):
    global simulated
    global animate

    if not simulated:
        # TODO make mbhp call to move robot to pose. does it go exactly where
        # we want? if not, adjust with twists.
        pass

    if simulated:
        if animate:
            cur_2dpose = Get2dPose(robot.GetTransform())
            step = (Get2dPose(pose)-cur_2dpose)/50.0
            for i in range(50):
                time.sleep(0.1)
                robot.SetTransform(GetTransform(cur_2dpose+step*i))
        robot.SetTransform(pose)

    # FIXME This should be removed when the robot controller updates the robot
    # pose from the real robot. 
    if not simulated:
        robot.SetTransform(pose)


def MoveBases(robots,poses):
    for (robot,pose) in izip(robots,poses):
        MoveBase(robot,pose)


# FIXME The planning is very dumb now. 
def PlanAssemblyOperation(robots,assembly_parts,assembly_parts_relative_poses):
    assembly_pose = GetAssemblyPose()

    assembly_robots = robots[:3] 
    
    # back
    armconfig_for_back = np.array([ 0.    ,  0.7398,  1.0021, -0.1783,  0.    ,  0.    ,  0.    ])
    manip_in_back = np.array([[ 0.0,  1.0,  0.0,  0.0006],
                                    [ 0.0,  0.0, -1.0,  0.0852],
                                    [-1.0,  0.0,  0.0,  0.0999],
                                    [ 0.0,  0.0,  0.0,  1.0   ]])

    # side
    #armconfig_for_side = np.array([ 0.    ,  1.2448,  0.9938, -0.6025,  0.    , -0.    ,  0.    ])
    armconfig_for_side = np.array([ 0.    ,  1.2448,  0.9362, -0.6025,  0.    ,  0.    ,  0.    ])
    manip_in_side = np.array([[ 0.0,  0.0, -1.0,  0.1123],
                              [-0.0, -1.0,  0.0, -0.0022],
                              [-1.0,  0.0,  0.0,  0.1112],
                              [ 0.0,  0.0,  0.0,  1.0   ]])

    # fastener
    armconfig_for_fastener = np.array([ 0.    ,  0.6699,  0.872 ,  0.0336,  0.    , -0.    , -0.    ])
    manip_in_fastener = np.array([[ 0.0, -1.0,  0.0, -0.002 ],
                                  [ 0.0,  0.0,  1.0, -0.0109],
                                  [-1.0,  0.0,  0.0,  0.0018],
                                  [ 0.0,  0.0,  0.0,  1.    ]])

    assembly_arm_configs = [armconfig_for_back, armconfig_for_side, armconfig_for_fastener]
    assembly_grasps = [manip_in_back, manip_in_side, manip_in_fastener] 

    assembly_base_poses = []
    for (robot,config,manip_in_part,part_relative_pose) in izip(assembly_robots,assembly_arm_configs,assembly_grasps,assembly_parts_relative_poses):
        with robot:
            robot.SetDOFValues(config)
            manip_in_world = robot.GetManipulators()[0].GetEndEffectorTransform()
            base_in_world = robot.GetTransform()
            manip_in_base = np.dot(invert_H(base_in_world),manip_in_world)
            part_in_base = np.dot(manip_in_base,invert_H(manip_in_part))
            part_in_world_for_assembly = np.dot(assembly_pose,part_relative_pose)
            base_in_world_for_assembly = np.dot(part_in_world_for_assembly, invert_H(part_in_base))
            assembly_base_poses.append(base_in_world_for_assembly)

    return (assembly_pose, assembly_robots, assembly_base_poses, assembly_arm_configs, assembly_grasps)


# vector(unitdirection*distance) is in robot frame and is 2d (x,y)
def Drive(robot,vector):
    global simulated
    global animate
    if simulated:
        pose = robot.GetTransform()
        pose[:2,3] = pose[:2,3] + pose[:2,0]*vector[0] + pose[:2,1]*vector[1]
        if animate:
            MoveBase(robot,pose)
        else:
            robot.SetTransform(pose)
    else:
        pass # TODO


def MoveToAssembly(robots,base_poses,arm_configs):
    # Move parts overhead so that they stay out of the way during navigation.
    for robot in robots:
        TuckArm(robot)

    # FIXME For now just goes to an intermediate pullback pose, and then moves straight.
    # compute pull-back base poses.
    pullback_distance = 1.0 # FIXME how to decide on this?
    for (robot,base_pose) in izip(robots,base_poses):
        forward_at_assembly = base_pose[:3,0]
        pullback_pose = base_pose
        pullback_pose[:3,3] = pullback_pose[:3,3] - pullback_distance*forward_at_assembly
        MoveBase(robot,pullback_pose)

    # Move arms to the assembly config
    for (robot,config) in izip(robots,arm_configs):
        MoveArm(robot,config)

    # Move base straight
    for robot in robots:
        Drive(robot,vector=np.array([1.0,0,0])*pullback_distance)


simulated = True
animate = True

env = orpy.Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)
env.Load('/home/mdogar/drl-youbot-openrave/models/environments/fouryoubots.env.xml') 
#env.SetDebugLevel(orpy.DebugLevel.Debug);
allrobots = env.GetRobots()

if not simulated:
    # Move robots to starting poses. Make sure they are close-by.
    # FIXME Make this a parameter.
    initial_robot_poses = [] # TODO Fill this in. What are good poses in holodeck?
    MoveBases(allrobots,initial_robot_poses) 

(assembly_parts,assembly_parts_relative_poses) = GetAssemblyStructure('chair_back_and_side')
(assembly_pose,assembly_robots,assembly_base_poses,assembly_arm_configs,assembly_grasps) = PlanAssemblyOperation(allrobots,assembly_parts,assembly_parts_relative_poses)
#TransformAssembly(assembly_structure, assembly_pose)
GraspAssemblyParts(assembly_robots,
                   assembly_arm_configs, # FIXME for now using the assembly joint angles for initial grasp. We probably should use different joint angles.
                   assembly_grasps,
                   assembly_parts)
MoveToAssembly(assembly_robots,assembly_base_poses,assembly_arm_configs)

# TODO update part grasps from vicon and robot joints at the beginning.
#      1. bi fonksiyon yaz, entera basinca ellerin, partlarin, ve robotlarin poselarini alsin vicondan/tfden (yeni tf plugini kullanabilirim). buna gore grasp'i(yukardaki manip_in_part'lar) set etsin.  joint anglelari de dinliyim (dalitsonun plugin?). holodeckte bag file kaydet. base transformunu cikar (bu bi kerelik is).  
#       1.1. once burda her zaman olcak kod: non simulated durumda partlarin ve robotun poselarini alip, robotun joint anglelarindan manip_in_part'i cikaran kodu yaz. 
#      2. bunlari yaptiktan sonra real robot commandlarini bagla bu scriptin, ve boylece robotlar birbirine yaklasabilsin assembly pose'a.
#       
#      Then switch to sundvik parts, choose (close) grasps
#           put in asm structure for sundvik parts
#           determine asm pose for sundvik parts in the env. A few of them..
#           generate ik solver
#           simulate all this?

# NEW TODO: 
#   1. grasp selection implement et. dusun basitce ne yapilabilir. tsr gibi bisey?
#     1.1 bi grasp sampler yazmam lazim. cont bi description olsa iyi olur.  linelar directionlar, widthler mi denesem?
#   2. naive search'u implement et bisuru forlarla.
#     2.1 arada ik gerekiyosa onu hallet.
#   3. bak bakalim nasil, dusun..



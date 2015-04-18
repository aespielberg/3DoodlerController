#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import random
from itertools import izip

import openravepy as orpy


#class AssemblyPoseSaver(object):
#    def __init__(self, assembly):
#        self.assembly = assembly
#    def __enter__(self):
#        self.saved_poses = []
#        for part in self.assembly.part_list:
#            self.saved_poses.append(part.body.GetTransform())
#        return 
#    def __exit__(self, type, value, traceback):
#        for part,pose in izip(self.assembly.part_list,self.saved_poses):
#            part.body.SetTransform(pose)
#        return 
#

#class PartGrasp(object):
#    def __init__(self,part_name,grasp):
#        self.part_name = part_name
#        self.grasp = grasp


#class Assembly(object):
#    def __init__(self, part_list, pose_list,name):
#        if len(part_list) != len(pose_list):
#            raise Exception('#parts do not match #poses.')
#        self.part_list = part_list
#        self.pose_per_part = pose_list
#        self.part_grasps = [] # list of list of grasps for each part. In part frame.
#        self.name = name
#
#    def CheckCollision(self,env,body):
#        for part in self.part_list:
#            if env.CheckCollision(body,part.body):
#                return True
#        return False
#
#    def SetTransform(self,asm_pose):
#       for part,pose_in_asm in izip(self.part_list,self.pose_per_part):
#           part.body.SetTransform(np.dot(asm_pose,pose_in_asm))
#       return
#
#    def Includes(self,part_name):
#        for part in self.part_list:
#            if part_name == part.name:
#                return True
#        return False
#
#    def GetPartPoseInAssembly(self,part_name):
#        for part,pose in izip(self.part_list,self.pose_per_part): 
#            if part.name == part_name:
#                return pose
#        raise Exception('Cannot find part with name {0}.'.format(part_name))


#class Part(Assembly):
#
#    def __init__(self,name):
#        Assembly.__init__(self,[self],[np.eye(4)],name)
#        self.name = name
#        self.body = None
#        self.grasps = None 
#
#    def SetGrasps(self,grasps):
#        self.grasps = grasps
#        for g in self.grasps:
#            self.part_grasps.append(PartGrasp(self.name,g))

    #def GenerateGrasps(self,env,hand):
    #    if self.grasps is None:
    #        part_grasp_generator = GraspGenerator.GraspGenerator()
    #        for part in assembly.part_list:
    #            self.grasps = part_grasp_generator.GetGrasps(part)
    #    for g in self.grasps:
    #        self.part_grasps.append(PartGrasp(self.name,g))


## old assembly structure below.
#class Assembly(object):
#
#    def __init__(self, subassembly_list, pose_list):
#        if len(subassembly_list) != len(pose_list):
#            raise Exception('#subassemblies do not match #poses.')
#        self.subassembly_list = subassembly_list
#        self.pose_list = pose_list
#        self.part_list = [] # lowest level parts
#        self.part_grasps = {} # dictionary of lowest level parts to grasps. 
#
#    def ExtractParts(self):
#        self.part_list = []
#        for each in self.subassembly_list:
#            self.part_list.extend(each.ExtractParts())
#        return self.part_list
#
#    def GenerateGrasps(self,robot):
#        subasm_grasps = []
#        for subasm,subasm_pose in izip(assembly.subassembly_list,assembly.pose_list):
#            for subasm_g in subasm.GetGrasps(robot):
#                g = np.dot(subasm_pose,subasm_g)
#                subasm_grasps.append(g)
#        asm_grasps = []
#        for g in subasm_grasps:
#            if self.IsPotentialGrasp(g):
#                asm_grasps.append(g)
#        self.
#        return asm_grasps
#
#    def IsPotentialGrasp(self,grasp,assembly):
#        robotinworld = robot.GetTransform()
#        worldinrobot = np.linalg.inv(robotinworld)
#        eeinworld = robot.GetManipulators()[0].GetEndEffectorTransform()
#        eeinrobot = np.dot(worldinrobot,eeinworld)
#        robotinee = np.linalg.inv(eeinrobot)
#        eeinw = grasp
#        robotinw = np.dot(eeinw,robotinee)
#        with robot:
#            robot.SetTransform(robotinw)
#            # TODO check hand links for collision against all other links of subassembly.
#
#
#class Part(object,Assembly):
#
#    def __init__(self,name,grasps=[],openrave_body=None):
#        Assembly.__init__(self,[],[])
#        self.name = name
#        self.grasps = grasps
#        self.openrave_body = openrave_body
#
#    def SetOpenraveBody(self,openrave_body):
#        self.openrave_body = openrave_body
#    
#    def ExtractParts(self): # The parts in this assembly is itself only.
#        return [self.name]
#
#    def GenerateGrasps(self,robot):
#        return self.grasps
#


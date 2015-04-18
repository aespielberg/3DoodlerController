#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import random
from itertools import izip
import IPython
import uuid
import copy

import openravepy as orpy

#from Assembly import PartGrasp,AssemblyPoseSaver
import AssemblyConstraint
import AssemblyGraspCSP
import CollisionConstraints

class AssemblyPoseSaver(object):
    def __init__(self, assembly):
        self.assembly = assembly
    def __enter__(self):
        self.saved_poses = []
        for part in self.assembly.GetPartList():
            self.saved_poses.append(part.body.GetTransform())
        return 
    def __exit__(self, type, value, traceback):
        for part,pose in izip(self.assembly.GetPartList(),self.saved_poses):
            part.body.SetTransform(pose)
        return 

class RobotGraspConfig(object):

    def __init__(self,robot_name,part_grasp,base_config,arm_config):
        self.robot_name = robot_name # TODO remove
        self.part_grasp = part_grasp
        self.arm_config = arm_config
        self.base_config = base_config
        self.unique_id = uuid.uuid4()

    def GetID(self):
        return self.unique_id

    def Visualize(self,env,robot_name):
        env.GetRobot(robot_name).SetDOFValues(self.arm_config,range(5))
        env.GetRobot(robot_name).SetTransform(self.base_config)
        pass


class AssemblyOperation(object):

    def __init__(self,assembly_list, assembly_poses,assembly_robot_names,name,order,robot_count=1):
        self.assembly_op_list = assembly_list
        self.assembly_op_poses = assembly_poses
        self.assembly_robot_names = assembly_robot_names
        self.constraint = None
        self.name = name
        self.robot_count = robot_count #TODO: instead, in the future, make this a list of regions.  Need to determine how to represent 3D regions.
        self.order = order
        self.h = []

    #def GetAssemblyConstraint(self,env,operation_pose,all_robot_names,all_object_names):
    #    part_names = []
    #    part_poses = []
    #    fixed_object_names = []
    #    for assembly,assembly_pose in izip(self.assembly_list,self.assembly_poses):
    #        for part,part_pose in izip(assembly.part_list,assembly.pose_per_part):
    #            part_names.append(part.name)
    #            part_poses.append(np.dot(operation_pose,np.dot(assembly_pose,part_pose))) 
    #    for object_name in all_object_names:
    #        if not object_name in part_names:
    #            part_names.append(object_name)
    #            part_poses.append(env.GetKinBody(object_name).GetTransform())
    #    return AssemblyConstraint.AssemblyConstraint(env,part_names,part_poses) 

    ## existing_robot_part_grasps is a list, including a tuple (robot_name,part_grasp) where part_grasp is a PartGrasp instance.
    #def PlanRobotGraspConfigs(self,env,yik,all_robot_names,all_object_names,operation_pose,randomize=False,existing_robot_part_grasps=[]):
    #    if self.constraint is None:
    #        self.constraint = self.GetAssemblyConstraint(env,operation_pose,all_robot_names,all_object_names)
    #    robot_grasp_configs = {}
    #    existing_robot_names = [rpg[0] for rpg in existing_robot_part_grasps]
    #    available_robot_names = [n for n in self.assembly_robot_names if not n in existing_robot_names]
    #    used_robot_names = []
    #    #print 'INITIALLY available are: '
    #    #for n in available_robot_names:
    #    #     print n,' '
    #    for assembly,assembly_pose in izip(self.assembly_list,self.assembly_poses):
    #        existing_grasp = False
    #        for existing_robot_part_grasp in existing_robot_part_grasps:
    #            if assembly.Includes(existing_robot_part_grasp[1].part_name):
    #                existing_grasp = True
    #                robot_name = existing_robot_part_grasp[0]
    #                assembly_part_grasps = [existing_robot_part_grasp[1]]
    #                break # only one existing grasp allowed
    #        if not existing_grasp:
    #            robot_name = available_robot_names[-1]
    #            del available_robot_names[-1]
    #            assembly_part_grasps = assembly.part_grasps
    #        all_robot_grasp_configs = self.GetRobotGraspConfigsGivenPartGrasps(env,yik,robot_name,assembly_part_grasps,assembly,assembly_pose,operation_pose)
    #        robot_grasp_configs[robot_name] = self.RemoveInconsistentRobotConfigs(robot_name,all_robot_grasp_configs)
    #        print 'robot',robot_name,' is holding ',assembly.name
    #        #print 'available are: '
    #        #for n in available_robot_names:
    #        #    print n,' '
    #        used_robot_names.append(robot_name)
    #    if randomize:
    #        for robot_name in robot_grasp_configs:
    #            #print robot_name+' nvals: '+str(len(robot_grasp_configs[robot_name]))
    #            random.shuffle(robot_grasp_configs[robot_name])
    #    result = AssemblyGraspCSP.BacktrackingSearch(used_robot_names,robot_grasp_configs,self.constraint)
    #    return result

    def GetRobotGraspConfigsGivenPartGrasps(self,env,yik,robot_name,part_grasps,assembly_op,assembly_op_pose,operation_pose):
        robot_grasp_configs = []
        for part_grasp in part_grasps:
            ee_pose = np.dot(operation_pose,np.dot(assembly_op_pose,np.dot(assembly_op.GetPartPoseInAssembly(part_grasp.part_name),part_grasp.grasp)))
            #if part_grasp.part_name == 'fastener_rightside_to_back':
            #    self.h.append(orpy.misc.DrawAxes(env,ee_pose,0.25,2.0))
            base_configs,arm_configs = self.SampleBaseAndArmGivenEEPose(env.GetRobot(robot_name),yik,ee_pose,returnfirst=False,rotationresolution=0.5,translationresolution=0.2)
            if base_configs is not None:
                for bc,ac in izip(base_configs,arm_configs):
                    robot_grasp_configs.append(RobotGraspConfig(robot_name,part_grasp,bc,ac))
        return robot_grasp_configs

    def GetRobotGraspConfigsSampled(self,env,yik,robot_name,child_assembly_op,operation_pose):
        robot_grasp_configs = []
        n_grasps = 0
        for part in child_assembly_op.GetPartList():
            if (not (type(child_assembly_op) is Part)) and not part.is_transferable:
                continue
            n_grasps += len(part.grasps)

        MAX_GRASPS = 50.0

        ratio = MAX_GRASPS / n_grasps

        for part in child_assembly_op.GetPartList():
            if (not (type(child_assembly_op) is Part)) and not part.is_transferable:
                continue
            n_grasps_for_this_part = int(ratio * len(part.grasps))
            sampled_grasps = random.sample(part.grasps,n_grasps_for_this_part)
            for grasp in sampled_grasps:
                ee_pose = np.dot(operation_pose,np.dot(self.GetPartPoseInAssembly(part.name),grasp))
                base_configs,arm_configs = self.SampleBaseAndArmGivenEEPose(env.GetRobot(robot_name),yik,ee_pose,returnfirst=False,rotationresolution=0.5,translationresolution=0.2,checkenvcollision=True)
                #base_configs,arm_configs = self.SampleBaseAndArmGivenEEPose(env.GetRobot(robot_name),yik,ee_pose,returnfirst=False,rotationresolution=0.5,translationresolution=0.2)
                if base_configs is not None:
                    for bc,ac in izip(base_configs,arm_configs):
                        robot_grasp_configs.append(RobotGraspConfig(robot_name,PartGrasp(part.name,grasp),bc,ac))
        #print child_assembly_op.name,' has ',len(robot_grasp_configs),' configs before filtering.'
        filtered_robot_grasp_configs = []
        for config in robot_grasp_configs:
            collision = False
            with env:
                with CollisionConstraints.RobotsConfig(env,
                                                       {robot_name:config.base_config},
                                                       {robot_name:config.arm_config},
                                                       range(5)):
                    for asm_op,asm_op_pose in izip(self.assembly_op_list,self.assembly_op_poses):
                        with AssemblyPoseSaver(asm_op):
                            asm_op.SetTransform(np.dot(operation_pose,asm_op_pose))
                            for part in asm_op.GetPartList():
                                if part.name == config.part_grasp.part_name:
                                    #print 'Disabling gripper collision.'
                                    self.EnableGripperLinks(env,robot_name,False)
                                collision = env.CheckCollision(env.GetRobot(robot_name),part.body)
                                if part.name == config.part_grasp.part_name:
                                    self.EnableGripperLinks(env,robot_name,True)
                                if collision:
                                    #print 'collision with ',part.name
                                    break
                        if collision:
                           break
                if not collision:
                    filtered_robot_grasp_configs.append(config)
        #print child_assembly_op.name,' has ',len(filtered_robot_grasp_configs),' configs after filtering.'
        return filtered_robot_grasp_configs


    def GetRobotGraspConfigs(self,env,yik,robot_name,child_assembly_op,operation_pose):
        robot_grasp_configs = []
        for part in child_assembly_op.GetPartList():
            if (not (type(child_assembly_op) is Part)) and not part.is_transferable:
                continue
            for grasp in part.grasps:
                ee_pose = np.dot(operation_pose,np.dot(self.GetPartPoseInAssembly(part.name),grasp))
                base_configs,arm_configs = self.SampleBaseAndArmGivenEEPose(env.GetRobot(robot_name),yik,ee_pose,returnfirst=False,rotationresolution=0.5,translationresolution=0.2,checkenvcollision=True)
                #base_configs,arm_configs = self.SampleBaseAndArmGivenEEPose(env.GetRobot(robot_name),yik,ee_pose,returnfirst=False,rotationresolution=0.5,translationresolution=0.2)
                if base_configs is not None:
                    for bc,ac in izip(base_configs,arm_configs):
                        robot_grasp_configs.append(RobotGraspConfig(robot_name,PartGrasp(part.name,grasp),bc,ac))
        #print child_assembly_op.name,' has ',len(robot_grasp_configs),' configs before filtering.'
        filtered_robot_grasp_configs = []
        for config in robot_grasp_configs:
            collision = False
            with env:
                with CollisionConstraints.RobotsConfig(env,
                                                       {robot_name:config.base_config},
                                                       {robot_name:config.arm_config},
                                                       range(5)):
                    for asm_op,asm_op_pose in izip(self.assembly_op_list,self.assembly_op_poses):
                        with AssemblyPoseSaver(asm_op):
                            asm_op.SetTransform(np.dot(operation_pose,asm_op_pose))
                            for part in asm_op.GetPartList():
                                if part.name == config.part_grasp.part_name:
                                    #print 'Disabling gripper collision.'
                                    self.EnableGripperLinks(env,robot_name,False)
                                collision = env.CheckCollision(env.GetRobot(robot_name),part.body)
                                if part.name == config.part_grasp.part_name:
                                    self.EnableGripperLinks(env,robot_name,True)
                                if collision:
                                    #print 'collision with ',part.name
                                    break
                        if collision:
                           break
                if not collision:
                    filtered_robot_grasp_configs.append(config)
        #print child_assembly_op.name,' has ',len(filtered_robot_grasp_configs),' configs after filtering.'
        return filtered_robot_grasp_configs

    def EnableGripperLinks(self,env,robot_name,enable):
        rob = env.GetRobot(robot_name)
        rob.GetLink('leftgripper').Enable(enable)
        rob.GetLink('rightgripper').Enable(enable)
        rob.GetLink('link5').Enable(enable)

    def GetNonCollidingRobotGraspConfigs(self,env,yik,robot_name,part_grasp,assembly_op,assembly_op_pose,operation_pose):
        robot_grasp_configs = self.GetRobotGraspConfigsGivenPartGrasps(env,yik,robot_name,[part_grasp],
                                                                       assembly_op,assembly_op_pose,operation_pose)
        filtered_robot_grasp_configs = []
        for config in robot_grasp_configs:
            collision = False
            with env:
                with CollisionConstraints.RobotsConfig(env,
                                                       {robot_name:config.base_config},
                                                       {robot_name:config.arm_config},
                                                       range(5)):
                    for asm_op,asm_op_pose in izip(self.assembly_op_list,self.assembly_op_poses):
                        with AssemblyPoseSaver(asm_op):
                            asm_op.SetTransform(np.dot(operation_pose,asm_op_pose))
                            for part in asm_op.GetPartList():
                                if part.name == part_grasp.part_name:
                                    self.EnableGripperLinks(env,robot_name,False)
                                collision = env.CheckCollision(env.GetRobot(robot_name),part.body)
                                if part.name == part_grasp.part_name:
                                    self.EnableGripperLinks(env,robot_name,True)
                                if collision:
                                    break
                        if collision:
                           break
                if not collision:
                    filtered_robot_grasp_configs.append(config)
        #print 'part grasps for asm op: ',part_grasp.part_name,' ',len(filtered_robot_grasp_configs)
        return filtered_robot_grasp_configs

    def RemoveInconsistentRobotConfigs(self,robot_name,robot_configs):
        new_robot_configs = []
        #print robot_name,': starts with ',len(robot_configs),' configs'
        for q in robot_configs:
            if self.constraint.IsConsistentUnary(robot_name,q):
                new_robot_configs.append(q)
        #print robot_name,': returns with ',len(new_robot_configs),' configs'
        return new_robot_configs

    def SampleBaseAndArmGivenEEPose(self,rob,yik,eepose,returnfirst=True,rotationresolution=0.05,translationresolution=0.01,checkenvcollision=False):
        #start = time.time()
        #print 'starting ik generation'
        self.EnableGripperLinks(rob.GetEnv(),rob.GetName(),False)
        base_poses,arm_configs=yik.FindIKAndBaseSolutions(rob,eepose,returnfirst=returnfirst,checkenvcollision=checkenvcollision,randomize=True,rotationresolution=rotationresolution,translationresolution=translationresolution)
        self.EnableGripperLinks(rob.GetEnv(),rob.GetName(),True)
        #print "IK generation took %f sec."%(time.time()-start)
        if base_poses is None or len(base_poses) == 0:
            #print 'Failed to find ik.'
            return None,None
        if returnfirst:
            randindex = random.randint(0,len(base_poses)-1)
            base_pose = base_poses[randindex]
            arm_config = np.array(arm_configs[randindex])
            return base_pose,arm_config
        else:
            return base_poses,arm_configs

    def IncludesPart(self,part_name):
        for asm_op in self.assembly_op_list:
            if asm_op.IncludesPart(part_name):
                return True
        return False

    def GetPartList(self): 
        part_list = []
        for asm_op in self.assembly_op_list:
             part_list.extend(asm_op.GetPartList())
        return part_list

    def GetPartAssemblyAndAssemblyPose(self,part_name):
        for asm_op,asm_op_pose in izip(self.assembly_op_list,self.assembly_op_poses):
            if asm_op.IncludesPart(part_name):
                return asm_op,asm_op_pose

    def GetPartPoseInAssembly(self,part_name):
        for asm_op,asm_op_pose in izip(self.assembly_op_list,self.assembly_op_poses):
            if asm_op.IncludesPart(part_name):
                return np.dot(asm_op_pose,asm_op.GetPartPoseInAssembly(part_name))

    def Visualize(self,operation_pose):
        self.SetTransform(operation_pose)

    def SetTransform(self,pose):
        for assembly_op,assembly_op_pose in izip(self.assembly_op_list,self.assembly_op_poses):
            assembly_op.SetTransform(np.dot(pose,assembly_op_pose))


class PartGrasp(object):
    def __init__(self,part_name,grasp):
        self.part_name = part_name
        self.grasp = grasp

class Part(AssemblyOperation):
    def __init__(self,name,is_transferable=True,robot_count=1):
        AssemblyOperation.__init__(self,[self],[np.eye(4)],[],name,[name],robot_count)
        self.name = name
        self.body = None
        self.grasps = None 
        self.is_transferable = is_transferable

    def SetGrasps(self,grasps):
        self.grasps = grasps
        #print self.name,'has ',len(grasps),' grasps.'
        #for g in self.grasps:
        #    self.part_grasps.append(PartGrasp(self.name,g))

    def SetTransform(self,pose):
        self.body.SetTransform(pose)

    def IncludesPart(self,name):
        return self.name == name

    def GetPartPoseInAssembly(self,name):
        if not (self.name == name):
            raise Exception('this should not happen')
        return np.eye(4)

    def GetPartList(self):
        return [self]


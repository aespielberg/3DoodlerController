#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import random
from itertools import izip

import openravepy as orpy

class RobotsConfig(object):
    def __init__(self,env,base_configs,arm_configs,armindices):
        self.new_bc = base_configs
        self.new_ac = arm_configs
        self.armindices = armindices
        self.env = env
        self.saved_bc = {}
        self.saved_ac = {}
    def __enter__(self):
        self.Save()
        self.Load(self.new_bc,self.new_ac)
    def __exit__(self, type, value, traceback):
        self.Load(self.saved_bc,self.saved_ac)
    def Save(self):
        self.saved_bc = {}
        self.saved_ac = {}
        for each in self.new_bc:
            self.saved_bc[each] = self.env.GetRobot(each).GetTransform()
            self.saved_ac[each] = self.env.GetRobot(each).GetDOFValues(self.armindices)
    def Load(self,bc,ac):
        for each in bc:
            self.env.GetRobot(each).SetTransform(bc[each])
            self.env.GetRobot(each).SetDOFValues(ac[each],self.armindices)


class AssemblyConstraint(object):
    def __init__(self,env,part_names,part_poses):
        self.env = env
        self.part_names = part_names
        self.part_poses = part_poses
        self.cache = {}

    # Checks only binary consistency between two variable value pairs.
    def IsConsistentBinary(self,var1,val1,var2,val2):
        if (not val1.GetID() in self.cache.keys()) or (not val2.GetID() in self.cache[val1.GetID()].keys()):
            collision = self.IsInCollision(val1.robot_name,val1.base_config,val1.arm_config,
                                           val2.robot_name,val2.base_config,val2.arm_config)
            consistent = not collision
            if val1.GetID() in self.cache.keys():
                self.cache[val1.GetID()][val2.GetID()] = consistent
            else:
                self.cache[val1.GetID()] = {val2.GetID():consistent}
            # Do the symmetric as well, because our constraints work symmetric.
            if val2.GetID() in self.cache.keys():
                self.cache[val2.GetID()][val1.GetID()] = consistent
            else:
                self.cache[val2.GetID()] = {val1.GetID():consistent}
        return self.cache[val1.GetID()][val2.GetID()]

    # Checks unary consistency.
    def IsConsistentUnary(self,var1,val1):
        # FIXME caching needed?
        collision = self.IsInWorldCollision(val1.robot_name,val1.base_config,val1.arm_config,[val1.part_grasp.part_name])
        consistent = not collision
        return consistent

    def EnableGripperLinks(self,env,robot_name,enable):
        rob = env.GetRobot(robot_name)
        rob.GetLink('leftgripper').Enable(enable)
        rob.GetLink('rightgripper').Enable(enable)
        rob.GetLink('link5').Enable(enable)

    def IsInWorldCollision(self,robot1_name,robot1_base_config,robot1_arm_config,exclude_objects=[]):
        base_configs = {robot1_name:robot1_base_config}
        arm_configs = {robot1_name:robot1_arm_config}
        collision = False
        with self.env:
            with RobotsConfig(self.env,base_configs,arm_configs,range(5)):
                for part_name,part_pose in izip(self.part_names,self.part_poses):
                    try:
                        if part_name in exclude_objects:
                            self.EnableGripperLinks(self.env,robot1_name,False)
                        with self.env.GetKinBody(part_name):
                            self.env.GetKinBody(part_name).SetTransform(part_pose)
                            collision = self.env.CheckCollision(self.env.GetRobot(robot1_name),self.env.GetKinBody(part_name))
                            if collision:
                                break
                    finally:
                        self.EnableGripperLinks(self.env,robot1_name,True)
        return collision

    def IsInCollision(self,robot1_name,robot1_base_config,robot1_arm_config,
                           robot2_name,robot2_base_config,robot2_arm_config):
        base_configs = {robot1_name:robot1_base_config,robot2_name:robot2_base_config}
        arm_configs = {robot1_name:robot1_arm_config,robot2_name:robot2_arm_config}
        with self.env:
            with RobotsConfig(self.env,base_configs,arm_configs,range(5)):
                #self.env.UpdatePublishedBodies()
                #time.sleep(0.01)
                return self.env.CheckCollision(self.env.GetRobot(robot1_name),self.env.GetRobot(robot2_name))




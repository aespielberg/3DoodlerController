#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import random
from itertools import izip

import openravepy as orpy

globalcache = {}

class BusrideConstraint(object):
    def __init__(self,var1,var2):
        self.var1 = var1
        self.var2 = var2

    def IsSameGrasp(self,g1,g2):
        diff = g1[:3,3] - g2[:3,3]
        #if np.linalg.norm(diff) < 0.001:
        if (diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2) > 0.00001:
            return False
        qd=orpy.quatMult(orpy.quatInverse(orpy.quatFromRotationMatrix(g1[:3,:3])),orpy.quatFromRotationMatrix(g2[:3,:3]))
        shortest_arc_angle = 2.0*np.arctan2(np.linalg.norm(qd[1:4]),qd[0])
        if shortest_arc_angle > 0.001:
            return False
        return True

    def IsFor(self,var):
        return self.var1 == var or self.var2 == var

    def IsConsistent(self,assignment):
        if (not (str(self.var1) in assignment.keys())) or (not (str(self.var2) in assignment.keys())):
               return True # don't return inconsistent if the assignment does not include the variables for this constraint.
        val1 = assignment[str(self.var1)]
        val2 = assignment[str(self.var2)]
        if (val1.part_grasp.part_name == val2.part_grasp.part_name) and self.IsSameGrasp(val1.part_grasp.grasp,val2.part_grasp.grasp):
            return True
        return False

    #def IsConsistent2(self,assignment):
    #    if not self.IsConsistent(assignment): # check just the grasp.
    #        return False
    #    # Check path-existence.
    #    # TODO run rrt planner (with no other robots, and no other parts in the future step (asm op imposes an order for the parts to approach))

    def __eq__(self,other):
        return (self.var1 == other.var1) and (self.var2 == other.var2)

    def __ne__(self,other):
        return not self.__eq__(other)

    def __hash__(self):
        return (hash(self.var1) ^ hash(self.var2))

class CollisionConstraints2(object):
    def __init__(self,env,robot_names,var1,var2):
        self.env = env
        self.robot_names = robot_names
        self.var1 = var1
        self.var2 = var2

    def __eq__(self,other):
        return (self.var1 == other.var1) and (self.var2 == other.var2)

    def __ne__(self,other):
        return not self.__eq__(other)

    def __hash__(self):
        return (hash(self.var1) ^ hash(self.var2))

    def IsFor(self,var):
        return self.var1 == var or self.var2 == var

    def IsConsistent(self,assignment):
        if (not (str(self.var1) in assignment.keys())) or (not (str(self.var2) in assignment.keys())):
               return True # don't return inconsistent if the assignment does not include the variables for this constraint.
        global globalcache
        config1 = assignment[str(self.var1)]
        config2 = assignment[str(self.var2)]
        # check if the result for these configs are not cached
        if (not config1.GetID() in globalcache.keys()) or (not config2.GetID() in globalcache[config1.GetID()].keys()):
            collision = self.CheckCollision(config1,config2)
            consistent = not collision
            # put the value in globalcache
            if config1.GetID() in globalcache.keys():
                globalcache[config1.GetID()][config2.GetID()] = consistent
            else:
                globalcache[config1.GetID()] = {config2.GetID():consistent}
            # Do the symmetric as well, because our constraints work symmetric.
            if config2.GetID() in globalcache.keys():
                globalcache[config2.GetID()][config1.GetID()] = consistent
            else:
                globalcache[config2.GetID()] = {config1.GetID():consistent}
        #else:
        #    print 'using cached value'
        return globalcache[config1.GetID()][config2.GetID()]

    #def IsConsistent2(self,assignment):
    #    return self.IsConsistent(assignment)

    def CheckCollision(self,config1,config2):
        # pick any two robots (XXX FIXME this works for now b/c all robots are same)
        robot1_name = self.robot_names[0]
        robot2_name = self.robot_names[1]
        # place the two robots at the config in vals 
        base_configs = {robot1_name:config1.base_config,
                        robot2_name:config2.base_config}
        arm_configs = {robot1_name:config1.arm_config,
                       robot2_name:config2.arm_config}
        with self.env:
            with RobotsConfig(self.env,base_configs,arm_configs,range(5)):
                # check openrave collision
                return self.env.CheckCollision(self.env.GetRobot(robot1_name),self.env.GetRobot(robot2_name))


class CollisionConstraints(object):
    def __init__(self,env,assembly_operations,robot_names,busride):
        self.env = env
        self.assembly_operations = assembly_operations # list of assembly operations
        self.robot_names = robot_names
        self.caches = {} 
        self.busride = busride
        for asm_op in assembly_operations:
            self.caches[asm_op.name] = {}

    def SetBusride(self,busride):
        self.busride = busride

    def IsConsistentBinary(self,var1,val1,var2,val2):
        # var1 and var2 are AssemblyCSPVariable objects
        # val1 and val2 are mappings from assembly_operation name to RobotGraspConfig
        for asm_op in self.assembly_operations:
            if ((var1.partname,var1.count) in self.busride[asm_op.name] 
               and (var2.partname,var2.count) in self.busride[asm_op.name]): # if both parts are grasped in this asm_op
                # check if the vals are consistent
                if not self.IsConsistentBinaryForAsmOp(asm_op,val1,val2):
                    return False
        return True

    def IsConsistentBinaryForAsmOp(self,asm_op,val1,val2):
        cache = self.caches[asm_op.name]
        config1 = val1[asm_op.name]
        config2 = val2[asm_op.name]
        # check if the result for these configs are not cached
        if (not config1.GetID() in cache.keys()) or (not config2.GetID() in cache[config1.GetID()].keys()):
            collision = self.CheckCollision(config1,config2)
            consistent = not collision
            # put the value in cache
            if config1.GetID() in cache.keys():
                cache[config1.GetID()][config2.GetID()] = consistent
            else:
                cache[config1.GetID()] = {config2.GetID():consistent}
           # Do the symmetric as well, because our constraints work symmetric.
            if config2.GetID() in cache.keys():
                cache[config2.GetID()][config1.GetID()] = consistent
            else:
                cache[config2.GetID()] = {config1.GetID():consistent}
        return cache[config1.GetID()][config2.GetID()]

    def CheckCollision(self,config1,config2):
        # pick any two robots (this works for now b/c all robots are same)
        robot1_name = self.robot_names[0]
        robot2_name = self.robot_names[1]
        # place the two robots at the config in vals 
        base_configs = {robot1_name:config1.base_config,
                        robot2_name:config2.base_config}
        arm_configs = {robot1_name:config1.arm_config,
                       robot2_name:config2.arm_config}
        with self.env:
            with RobotsConfig(self.env,base_configs,arm_configs,range(5)):
                # check openrave collision
                return self.env.CheckCollision(self.env.GetRobot(robot1_name),self.env.GetRobot(robot2_name))


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


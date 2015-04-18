#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import IPython
import random
import sys
import time
import copy
import operator
import uuid
from itertools import izip,product,combinations
from operator import itemgetter

import openravepy as orpy

from youbotpy import youbotik as yik
from tsr import GraspTSR,LookAtTSR
from wingdemo_misc import XYThetaToMatrix,WorldConfig,RobotsConfig,Grabbed,VisualizeAssemblyOperation,MakeRobotTransparentExceptHand,VisualizeAssemblyGrasp
from scipy.spatial.distance import cdist,pdist

class RobotGraspConfig(object):
    def __init__(self,robot_name,object_name,base_config,arm_config):
        self.robot_name = robot_name
        self.object_name = object_name
        self.arm_config = arm_config
        self.base_config = base_config
        self.unique_id = uuid.uuid4()

    def GetID(self):
        return self.unique_id

class AssemblyConstraints(object):
    def __init__(self,env,world):
        self.env = env
        self.world = world
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
        collision = self.IsInWorldCollision(self.world,val1.robot_name,val1.base_config,val1.arm_config,[val1.object_name])
        consistent = not collision
        return consistent

    def IsInWorldCollision(self,world,robot1_name,robot1_base_config,robot1_arm_config,exclude_objects=[]):
        base_configs = {robot1_name:robot1_base_config}
        arm_configs = {robot1_name:robot1_arm_config}
        collision = False
        with self.env:
            with RobotsConfig(self.env,base_configs,arm_configs,range(5)):
                for each in world['object_names']:
                    if not each in exclude_objects:
                        with self.env.GetKinBody(each):
                            self.env.GetKinBody(each).SetTransform(world['object_poses'][each])
                            #self.env.UpdatePublishedBodies()
                            #time.sleep(0.0005)
                            collision = self.env.CheckCollision(self.env.GetRobot(robot1_name),self.env.GetKinBody(each))
                            #if not robot1_name == 'drc1':
                            #    print 'Collision {0}'.format(collision)
                            #    raw_input('hit enter')
                            if collision:
                                break
                if not collision:
                    for each in world['fixed_objects']:
                        collision = self.env.CheckCollision(self.env.GetRobot(robot1_name),self.env.GetKinBody(each))
                        #if not robot1_name == 'drc1':
                        #    print 'Collision {0}'.format(collision)
                        #    raw_input('hit enter')
                        if collision:
                            break
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


def FindMinimumRemainingValuesVariable(variables,values):
    min_n_vals = sys.maxint 
    mrv_variable = None
    for var in variables:
        if len(values[var]) < min_n_vals:
            mrv_variable = var
            min_n_vals = len(values[var])
    return mrv_variable

def SelectUnassignedVariable(assignment,unassigned_variables,values):
    return FindMinimumRemainingValuesVariable(unassigned_variables,values)

def OrderDomainValues(var,assignment,values):
    # FIXME implement least-constraining-value?
    # FIXME implement distance heuristic here?
    # FIXME implement n-regrasp bias here?
    print 'returning values of length: ',len(values[var])
    return values[var]

def IsConsistent(var,value,assignment,constraints):
    for var2,value2 in assignment:
        if not constraints.IsConsistentBinary(var,value,var2,value2):
            return False
    return True

def FilterViaForwardChecking(var,value,unassigned_variables,values,constraints):
    filtered_values = {}
    for var2 in unassigned_variables:
        filtered_values[var2] = []
        for value2 in values[var2]:
            if constraints.IsConsistentBinary(var,value,var2,value2):
                filtered_values[var2].append(value2)
    return filtered_values

def RecursiveBacktracking(assignment,unassigned_variables,values,constraints):
    if len(unassigned_variables) == 0:
        return assignment
    var = SelectUnassignedVariable(assignment,unassigned_variables,values)
    print 'planning for ',var
    unassigned_variables = copy.copy(unassigned_variables)
    unassigned_variables.remove(var)
    for value in OrderDomainValues(var,assignment,values):
        #if IsConsistent(var,value,assignment,constraints): # I think not required since if are doing forward checking
            assignment.append((var,value))
            filtered_values = FilterViaForwardChecking(var,value,unassigned_variables,values,constraints)
            result = RecursiveBacktracking(copy.copy(assignment),unassigned_variables,filtered_values,constraints)
            if result is not None:
                return result
            del assignment[-1]
    print var, ': returning None'
    return None

def BacktrackingSearch(variables,values,constraints):
    return RecursiveBacktracking([],variables,values,constraints)



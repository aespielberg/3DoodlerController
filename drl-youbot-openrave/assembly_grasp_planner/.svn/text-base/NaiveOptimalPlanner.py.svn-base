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

from AssemblyOperation import PartGrasp,Part
import AssemblyGraspCSP
import BaseAssemblyGraspPlanner

class NaiveOptimalPlanner(BaseAssemblyGraspPlanner.BaseAssemblyGraspPlanner):
    def __init__(self):
        BaseAssemblyGraspPlanner.BaseAssemblyGraspPlanner.__init__(self)

    def Plan(self,env,yik,op_pose,assembly_operations,all_robot_names,time_budget_minutes=30.0):
        variables,asm_op_variables = self.ComputeVariables(assembly_operations)
        values = self.ComputeAllValues(env,yik,variables,op_pose)
        collision_constraints,collision_constraints_per_op = self.GenerateCollisionConstraints(env,all_robot_names,asm_op_variables)
        start_time = time.time()
        non_trivial_vars = [v for v in variables if not (type(v.parent_operation) is Part)]
        must_exit = False
        optimal_assignment = None
        for n_transfers in range(len(non_trivial_vars),-1,-1): # backwards until 0
            if time.time()-start_time > time_budget_minutes*60.0 or must_exit:
                break
            c_lists = self.GetNBusrideConstraintsSpace(n_transfers,assembly_operations,variables,asm_op_variables)
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
                    self.last_run_n_transfers = [n_transfers]
                    self.last_run_times = [time.time()-start_time]
                    must_exit = True
                    min_regrasp_assignment = optimal_assignment
                    min_regrasp_constraints = new_constraints
                    break
        if optimal_assignment is None:
            self.last_run_n_transfers = None
            self.last_run_times = None
            return None,None,None,None
        else:
            return min_regrasp_assignment,min_regrasp_constraints,variables,asm_op_variables 



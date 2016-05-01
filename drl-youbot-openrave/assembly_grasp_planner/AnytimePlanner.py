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

class AnytimePlanner(BaseAssemblyGraspPlanner.BaseAssemblyGraspPlanner):
    def __init__(self):
        BaseAssemblyGraspPlanner.BaseAssemblyGraspPlanner.__init__(self)

    def IncludesFailed(self,current,failed_list):
        for failed in failed_list:
            if set(failed) < set(current):
                return True
        return False

    def Plan(self,env,yik,op_pose,assembly_operations,all_robot_names,time_budget_minutes=30.0):
        variables,asm_op_variables = self.ComputeVariables(assembly_operations)
        values = self.ComputeAllValues(env,yik,variables,op_pose)
        collision_constraints,collision_constraints_per_op = self.GenerateCollisionConstraints(env,all_robot_names,asm_op_variables)
        start_time = time.time()
        all_regrasp_plan = {}
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
        
        if no_soln:
            self.last_run_n_transfers = None
            self.last_run_times = None
            # return failure because first csp did not work.
            return None,None,None,None
        else:
            print('Found solution without busride constraints. Now trying to fix regrasps.')
            for op in assembly_operations:
                for var_str in assignment_per_op[op.name].keys():
                    all_regrasp_plan[var_str] = assignment_per_op[op.name][var_str]
            min_regrasp_assignment = copy.copy(all_regrasp_plan)
            all_regrasp_constraints = copy.copy(collision_constraints)
            non_trivial_vars = [v for v in variables if not (type(v.parent_operation) is Part)]
            min_regrasp_constraints = None
            min_regrasp_c_list = []
            failed_list = []
            self.last_run_n_transfers = [0]
            self.last_run_times = [time.time()-start_time]
            for n_transfers in range(1,len(non_trivial_vars)+1):
                if time.time()-start_time > time_budget_minutes*60.0:
                    break
                c_lists = self.GetNBusrideConstraintsSpace(n_transfers,assembly_operations,variables,asm_op_variables)
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
                    #if not (self.IncludesFailed(c_list,failed_list)): # pruning
                    if True: # pruning
                        new_constraints = copy.copy(all_regrasp_constraints)
                        new_constraints.extend(c_list)
                        new_assignment = copy.copy(min_regrasp_assignment)
                        print 'one minconflict attempt. n: ',n_transfers
                        success = AssemblyGraspCSP.MinConflict(variables,values,new_constraints,new_assignment,max_steps=60*n_transfers)
                        print 'minconflict attempt done!'
                        if success:
                            self.last_run_n_transfers.append(n_transfers) 
                            self.last_run_times.append(time.time()-start_time)
                            print 'solution with ',n_transfers,' busride constraints found!'
                            min_regrasp_assignment = new_assignment
                            min_regrasp_constraints = new_constraints
                            min_regrasp_c_list = c_list
                            break
                        else:
                            failed_list.append(c_list)
                    else:
                        print 'constraint list pruned.'
        return min_regrasp_assignment,min_regrasp_constraints,variables,asm_op_variables 
  
  

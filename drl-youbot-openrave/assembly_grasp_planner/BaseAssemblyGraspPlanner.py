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
import CollisionConstraints

class BaseAssemblyGraspPlanner(object):
    def __init__(self):
        self.last_run_n_transfers = None
        self.last_run_times = None

    def GetLastRunRecords(self):
        return self.last_run_n_transfers,self.last_run_times
 
    def ComputeVariables(self,assembly_operations):
        variables = []
        asm_op_variables = {}
        for parent_asm_op in assembly_operations:
            asm_op_variables[parent_asm_op.name] = {}
            for asm_op in parent_asm_op.assembly_op_list:
                asm_op_variables[parent_asm_op.name][asm_op.name] = [] 
                for i in range(asm_op.robot_count):
                    v = AssemblyGraspCSP.AssemblyCSPVariable2(parent_asm_op,asm_op,i)
                    variables.append(v)
                    asm_op_variables[parent_asm_op.name][asm_op.name].append(v)
        return variables,asm_op_variables

    def ComputeValuesForVariable(self,v,env,yik,robot_name,op_pose):
        return v.parent_operation.GetRobotGraspConfigs(env,yik,robot_name,v.assembly,op_pose)
        #return v.parent_operation.GetRobotGraspConfigsSampled(env,yik,robot_name,v.assembly,op_pose)

    def ComputeAllValues(self,env,yik,variables,op_pose):
        values = {}
        for v in variables:
            this_op_pose = op_pose
            if type(v.parent_operation) is Part:
                this_op_pose = env.GetKinBody(v.parent_operation.name).GetTransform() # Use current transform of part in openrave world.
            values[str(v)] = self.ComputeValuesForVariable(v,env,yik,'drc1',this_op_pose) # TODO fix name drc1
            #print str(v),' has ',len(values[str(v)]),' values'
        return values

    def GenerateCollisionConstraints(self,env,all_robot_names,asm_op_variables):
        constraints_per_op = {}
        constraints = []
        for asm_to_vars,op_name in izip(asm_op_variables.values(),asm_op_variables.keys()):
            constraints_per_op[op_name] = []
            for var1,var2 in combinations(chain.from_iterable(asm_to_vars.values()),2):
                c = CollisionConstraints.CollisionConstraints2(env,all_robot_names,var1,var2)
                constraints.append(c)
                constraints_per_op[op_name].append(c)
        return constraints,constraints_per_op
 
    def ContainsDuplicateVar2(self,c_list):
        for i,c in enumerate(c_list):
            for c_other in c_list[i+1:]:
                if c.var2 == c_other.var2:
                    return True
        return False
    
    def GetAllPossibleBusrideConstraintsForVars(self,variables,asm_op_variables):
        var_constraints = []
        for v in variables:
            if type(v.parent_operation) is Part:
                continue
            cur_var_constraints = []
            for subasm in v.assembly.assembly_op_list:
                #IPython.embed()
                for var2 in asm_op_variables[v.assembly.name][subasm.name]: # In case there are more than one grasp of this subasm, create a new constraint for each.
                    c = CollisionConstraints.BusrideConstraint(v,var2)
                    cur_var_constraints.append(c)
            var_constraints.append(cur_var_constraints)
        constraints = [c for c in product(*var_constraints)]
        # Filter constraints out if they include link to same var2. (necessary for multi-grasp of same assembly).
        constraints = [c for c in constraints if not self.ContainsDuplicateVar2(c)]
        #IPython.embed()
        return constraints
    
    # Returns the list consisting of sets of N busride constraints.
    def GetNBusrideConstraintsSpace(self,n,assembly_operations,variables,asm_op_variables):
        space = []
        # filter out trivial variables
        non_trivial_vars = [v for v in variables if not (type(v.parent_operation) is Part)]
        for n_comb in combinations(non_trivial_vars,n):
            new_space = self.GetAllPossibleBusrideConstraintsForVars(n_comb,asm_op_variables)
            space.extend(new_space)
        return space
 

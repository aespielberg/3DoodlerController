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

def FindMinimumRemainingValuesVariable(variables,values):
    min_n_vals = sys.maxint 
    mrv_variable = None
    for var in variables:
        if len(values[str(var)]) < min_n_vals:
            mrv_variable = var
            min_n_vals = len(values[str(var)])
    return mrv_variable

def SelectUnassignedVariable(assignment,unassigned_variables,values):
    return FindMinimumRemainingValuesVariable(unassigned_variables,values)

def OrderDomainValues(var,assignment,values):
    # FIXME implement least-constraining-value?
    # FIXME implement distance heuristic here?
    # FIXME implement n-regrasp bias here?
    #print 'returning values of length: ',len(values[str(var)])
    return values[str(var)]

#def IsConsistent(var,value,assignment,constraints):
#    for var2,value2 in assignment:
#        if not constraints.IsConsistentBinary(var,value,var2,value2):
#            return False
#    return True

def FilterViaForwardChecking(var,value,unassigned_variables,values,constraints):
    filtered_values = {}
    for var2 in unassigned_variables:
        filtered_values[str(var2)] = []
        for value2 in values[str(var2)]:
            if constraints.IsConsistentBinary(var,value,var2,value2):
                filtered_values[str(var2)].append(value2)
    return filtered_values

def RecursiveBacktracking(assignment,unassigned_variables,values,constraints):
    if len(unassigned_variables) == 0:
        return assignment
    var = SelectUnassignedVariable(assignment,unassigned_variables,values)
    #print 'planning for ',var
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
    #print var, ': returning None'
    return None

def BacktrackingSearch(variables,values,constraints):
    return RecursiveBacktracking([],variables,values,constraints)

## 2

def CheckConstraints(assignment,constraints):
    for c in constraints:
        if not c.IsConsistent(assignment):
            return False
    return True

def FilterViaForwardChecking2(var,value,unassigned_variables,values,constraints):
    filtered_values = {}
    for var2 in unassigned_variables:
        filtered_values[str(var2)] = []
        for value2 in values[str(var2)]:
            assignment = {str(var):value,str(var2):value2}
            if CheckConstraints(assignment,constraints):
                filtered_values[str(var2)].append(value2)
    return filtered_values

def IsAssignmentConsistent(assignment,constraints):
    for c in constraints:
        if not c.IsConsistent(assignment):
            return False
    return True

def IsConsistent2(var,value,assignment,constraints):
    new_assignment = copy.copy(assignment)
    new_assignment[str(var)] = value
    return IsAssignmentConsistent(new_assignment,constraints)

def RecursiveBacktracking2(assignment,unassigned_variables,values,constraints,timeout):
    if (not (timeout is None)) and (time.time() > timeout):
        return None
    if len(unassigned_variables) == 0:
        return assignment
    var = SelectUnassignedVariable(assignment,unassigned_variables,values)
    #print 'planning for ',str(var)
    unassigned_variables = copy.copy(unassigned_variables)
    unassigned_variables.remove(var)
    for value in OrderDomainValues(var,assignment,values):
        #if IsConsistent2(var,value,assignment,constraints): # I think not required since if are doing forward checking
            assignment[str(var)] = value
            filtered_values = FilterViaForwardChecking2(var,value,unassigned_variables,values,constraints)
            result = RecursiveBacktracking2(copy.copy(assignment),unassigned_variables,filtered_values,constraints,timeout)
            #result = RecursiveBacktracking2(copy.copy(assignment),unassigned_variables,values,constraints)
            if result is not None:
                return result
            del assignment[str(var)]
    #print str(var), ': returning None'
    return None

def BacktrackingSearch2(variables,values,constraints,timeout=None):
    return RecursiveBacktracking2({},variables,values,constraints,timeout)

###

def GetConflictingVariables(assignment,constraints):
    conflicting = []
    for c in constraints:
        if not c.IsConsistent(assignment):
            if not (c.var1 in conflicting):
                conflicting.append(c.var1)
            if not (c.var2 in conflicting):
                conflicting.append(c.var2)
    return conflicting

def GetNConflictingConstraints(var,assignment,constraints,threshold):
    n_conflicting = 0
    #t1 = time.time()
    for c in constraints:
        if c.IsFor(var):
            if not c.IsConsistent(assignment):
                n_conflicting += 1
                if n_conflicting > threshold: # We don't need to continue.
                    #print 'c.GetNConflictingConstraints() took ',time.time()-t1,'sec.'
                    return n_conflicting
    #print 'c.GetNConflictingConstraints() took ',time.time()-t1,'sec.'
    return n_conflicting

def GetConflictMinimizingValue(var,values,assignment,constraints,n_value_sample=30):
    n_conflicts = []
    assignment = copy.copy(assignment)
    min_conflicting = sys.maxint 
    if n_value_sample > len(values):
        sampled_values = values
    else:
        sampled_values = random.sample(values,n_value_sample)
    for val in sampled_values:
        assignment[str(var)] = val
        this_n = GetNConflictingConstraints(var,assignment,constraints,min_conflicting)
        n_conflicts.append(this_n)
        if this_n < min_conflicting:
            min_conflicting = this_n
    # Get all values with the min conflict
    min_conflict_vals = []
    for val,n_c in izip(sampled_values,n_conflicts):
        if min_conflicting == n_c:
            min_conflict_vals.append(val)
    return random.choice(min_conflict_vals)

def MinConflict(variables,values,constraints,assignment,max_steps):
    for i in range(max_steps):
        if IsAssignmentConsistent(assignment,constraints): 
            return True
        conflicting_variables = GetConflictingVariables(assignment,constraints)
        var = random.choice(conflicting_variables)
        t2 = time.time()
        value = GetConflictMinimizingValue(var,values[str(var)],assignment,constraints)
        print 'GetConflictMinimizingValue took ',time.time()-t2,'sec.'
        assignment[str(var)] = value
    return False

###

class AssemblyCSPVariable(object):
    def __init__(self,partname,count):
        self.partname = partname
        self.count = count
    def __str__(self):
        return str(self.partname)+'_'+str(self.count)
    def __eq__(self,other):
        return (self.partname == other.partname) and (self.count == other.count)
    def __ne__(self,other):
        return not self.__eq__(other)
    def __key(self):
        return (self.partname,self.count)
    def __hash__(self):
         return hash(self.__key())

class AssemblyCSPVariable2(object):
    def __init__(self,parent_operation,assembly,count):
        self.parent_operation = parent_operation
        self.assembly = assembly
        self.count = count
    def __str__(self):
        return str(self.parent_operation.name)+'__'+str(self.assembly.name)+'__'+str(self.count)
    def __eq__(self,other):
        return (self.parent_operation.name == other.parent_operation.name) and (self.assembly.name == other.assembly.name) and (self.count == other.count)
    def __ne__(self,other):
        return not self.__eq__(other)
    def __key(self):
        return (self.parent_operation.name,self.assembly.name,self.count)
    def __hash__(self):
         return hash(self.__key())

